#include <Arduino.h>
#include <driver/i2s_std.h>
#include <math.h>

// Используем встроенный в Meshtastic движок Nanopb
#include <pb_encode.h>
#include <pb_decode.h>

// Автоматически определяем правильный путь к структурам Meshtastic
#if __has_include(<meshtastic/mesh.pb.h>)
  #include <meshtastic/mesh.pb.h>
#else
  #include <mesh.pb.h>
#endif

#define I2S_BCLK_PIN GPIO_NUM_41
#define I2S_WS_PIN   GPIO_NUM_42
#define I2S_DATA_PIN GPIO_NUM_40

// Настройка пинов UART для ESP32-S3
#define ESP_RX_PIN 16 
#define ESP_TX_PIN 17 

HardwareSerial EspSerial(2);

// ID целевой ноды (!a7eef537)
const uint32_t TARGET_NODE_ID = 0xA7EEF537;

#define SAMPLE_RATE     16000  
#define BLOCK_SIZE      1024   

// Сетка частот БПЛА
const float DRONE_FREQS[] = {
    140.0f, 150.0f, 160.0f, 170.0f, 180.0f, 190.0f, 
    200.0f, 210.0f, 220.0f, 230.0f, 240.0f, 250.0f
}; 
const int NUM_FREQS = 12; 

// Порог SNR (в dB) для фиксации частоты
const float SNR_THRESHOLD_DB = 3.0f;  

// --- НАСТРОЙКИ ДИНАМИЧЕСКОГО ШУМОПОДАВЛЕНИЯ ---
float dynamic_volume_floor = 150.0f;  // Начальное значение уровня фонового шума
const float NOISE_MARGIN = 80.0f;     // Запас над фоном (порог = фон + запас)

// Раздельная адаптация: вверх медленно, вниз быстро (быстро возвращает чувствительность)
const float ADAPT_RATE_UP = 0.005f;   
const float ADAPT_RATE_DOWN = 0.05f;  

// --- УСИЛЕННАЯ ВАЛИДАЦИЯ ВО ВРЕМЕНИ ---
#define REQUIRED_STREAK 10     
#define TOTAL_WINDOWS   14      

class DroneValidator {
private:
    bool history[TOTAL_WINDOWS] = {false};
    int insert_index = 0;
public:
    void update(bool detected) {
        history[insert_index] = detected;
        insert_index = (insert_index + 1) % TOTAL_WINDOWS;
    }
    bool is_confirmed() {
        int count = 0;
        for (int i = 0; i < TOTAL_WINDOWS; i++) {
            if (history[i]) count++;
        }
        return (count >= REQUIRED_STREAK);
    }
};
DroneValidator validator;

i2s_chan_handle_t rx_handle = NULL;
int16_t audio_buffer[BLOCK_SIZE];
bool drone_was_detected = false;

// Таймеры для отладки и ограничения линии связи
unsigned long last_debug_time = 0;
unsigned long last_alarm_send_time = 0;
const unsigned long ALARM_INTERVAL_MS = 10000; // Ограничение отправки сообщений: раз в 10 секунд

void init_i2s() {
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    i2s_new_channel(&chan_cfg, NULL, &rx_handle);

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCLK_PIN,
            .ws = I2S_WS_PIN,
            .dout = I2S_GPIO_UNUSED,
            .din = I2S_DATA_PIN,
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false }
        }
    };
    
    std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT; 
    i2s_channel_init_std_mode(rx_handle, &std_cfg);
    i2s_channel_enable(rx_handle);
}

// Оптимизированный расчет на int16_t без деления внутри цикла (защита от Underflow во float)
float goertzel_energy(const int16_t* samples, int num_samples, float target_freq, float sampling_rate) {
    float k = 0.5f + ((float)num_samples * target_freq) / sampling_rate;
    float omega = (2.0f * M_PI / (float)num_samples) * k;
    float cosine = cosf(omega);
    float coefficient = 2.0f * cosine;

    float q0 = 0.0f, q1 = 0.0f, q2 = 0.0f;

    for (int i = 0; i < num_samples; i++) {
        q0 = (float)samples[i] + coefficient * q1 - q2;
        q2 = q1;
        q1 = q0;
    }
    float energy = (q1 * q1) + (q2 * q2) - (coefficient * q1 * q2);
    
    // Масштабируем результат один раз на выходе (32768^2)
    return energy / 1073741824.0f;
}

float get_snr(const int16_t* samples, int num_samples, float target_freq) {
    float target_energy = goertzel_energy(samples, num_samples, target_freq, SAMPLE_RATE);
    
    // Шумовые карманы вынесены за пределы сетки частот, чтобы избежать взаимного подавления
    float noise_l = goertzel_energy(samples, num_samples, 110.0f, SAMPLE_RATE);
    float noise_r = goertzel_energy(samples, num_samples, 280.0f, SAMPLE_RATE);
    float avg_noise = (noise_l + noise_r) / 2.0f;

    if (avg_noise <= 0.0f) avg_noise = 0.00001f;
    return 10.0f * log10f(target_energy / avg_noise);
}

float calculate_rms(const int16_t* samples, int num_samples) {
    double sum = 0;
    for (int i = 0; i < num_samples; i++) {
        sum += (double)samples[i] * samples[i];
    }
    return sqrt(sum / num_samples);
}

void sendDirectMessage(const char* text, uint32_t targetId) {
  uint8_t buffer[256]; // Выделяем массив под бинарные данные пакета
  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

  // Инициализируем корневую структуру контейнера ToRadio
  meshtastic_ToRadio toRadioPacket = meshtastic_ToRadio_init_zero;
  toRadioPacket.which_payload_variant = meshtastic_ToRadio_packet_tag; 

  // Прямое заполнение структуры MeshPacket внутри контейнера ToRadio
  toRadioPacket.packet.to = targetId;   // ID получателя (0xC1E3B1C4)
  toRadioPacket.packet.from = 0;        // 0 заставит ноду подставить свой собственный ID
  toRadioPacket.packet.want_ack = true; // Запрашиваем аппаратный ACK (подтверждение доставки)

  // Настройка полезной нагрузки сообщения (Decoded Data)
  toRadioPacket.packet.which_payload_variant = meshtastic_MeshPacket_decoded_tag;
  toRadioPacket.packet.decoded.portnum = meshtastic_PortNum_TEXT_MESSAGE_APP; // Порт 1: Обычный текст

  // Копируем текст сообщения "TEST" в массив payload
  size_t text_len = strlen(text);
  if (text_len > sizeof(toRadioPacket.packet.decoded.payload.bytes)) {
    text_len = sizeof(toRadioPacket.packet.decoded.payload.bytes);
  }
  memcpy(toRadioPacket.packet.decoded.payload.bytes, text, text_len);
  toRadioPacket.packet.decoded.payload.size = text_len;

  // Кодируем структуру ToRadio встроенными средствами Nanopb
  if (!pb_encode(&stream, meshtastic_ToRadio_fields, &toRadioPacket)) {
    Serial.print("Ошибка упаковки Protobuf: ");
    Serial.println(PB_GET_ERROR(&stream));
    return;
  }

  size_t packet_length = stream.bytes_written;

  // Кадрирование бинарного пакета по стандарту Meshtastic Serial Framer
  // Отправляем маркеры начала [0x94] [0xC3] и 2 байта длины пакета
  EspSerial.write(0x94);
  EspSerial.write(0xC3);
  EspSerial.write((packet_length >> 8) & 0xFF);
  EspSerial.write(packet_length & 0xFF);
  
  // Передаем готовый бинарный Protobuf-пакет в Serial-шину Heltec
  EspSerial.write(buffer, packet_length);

  Serial.println("Бинарный DM пакет (ToRadio) успешно отправлен на ноду A7EEF537.");
}

void setup() {
    Serial.begin(115200);
    init_i2s();
    Serial.println("Система с адаптивным шумоподавлением запущена...");
    last_alarm_send_time = millis() - ALARM_INTERVAL_MS; // Разрешаем отправку первого сообщения сразу

    // В бинарном режиме PROTO Meshtastic работает строго на 115200 бод
    EspSerial.begin(115200, SERIAL_8N1, ESP_RX_PIN, ESP_TX_PIN);
    delay(1000);

    // Синхронизируем Serial-интерфейс стартовыми wake-байтами
    for(int i = 0; i < 4; i++) {
        EspSerial.write(0x94);
  }

   Serial.println("ESP32 Мастер (Встроенный API Meshtastic/Nanopb) запущен.");

}

void loop() {
    size_t bytes_read = 0;
    esp_err_t res = i2s_channel_read(rx_handle, audio_buffer, sizeof(audio_buffer), &bytes_read, portMAX_DELAY);
    
    if (res == ESP_OK && bytes_read > 0) {
        bool frame_has_drone = false;
        float current_rms = calculate_rms(audio_buffer, BLOCK_SIZE);
        
        float active_threshold = dynamic_volume_floor + NOISE_MARGIN;
        bool show_debug = (millis() - last_debug_time > 500);

        float max_snr = -99.0f;
        float best_freq = 0;

        // 1. Анализируем SNR только при превышении порога громкости
        if (current_rms > active_threshold) {
            for (int i = 0; i < NUM_FREQS; i++) {
                float snr = get_snr(audio_buffer, BLOCK_SIZE, DRONE_FREQS[i]);
                if (snr > max_snr) {
                    max_snr = snr;
                    best_freq = DRONE_FREQS[i];
                }
                if (snr > SNR_THRESHOLD_DB) {
                    frame_has_drone = true; 
                }
            }
        } 
        
        // 2. Двухскоростная адаптация фонового уровня шума (только если на кадре нет признаков дрона)
        if (!frame_has_drone) {
            if (current_rms > dynamic_volume_floor) {
                dynamic_volume_floor = (dynamic_volume_floor * (1.0f - ADAPT_RATE_UP)) + (current_rms * ADAPT_RATE_UP);
            } else {
                dynamic_volume_floor = (dynamic_volume_floor * (1.0f - ADAPT_RATE_DOWN)) + (current_rms * ADAPT_RATE_DOWN);
            }
        }

        // 3. Вывод отладочной информации в Serial (каждые 500 мс)
        if (show_debug) {
            if (current_rms > active_threshold) {
                Serial.print("Громкость: "); Serial.print(current_rms);
                Serial.print(" (Порог: "); Serial.print(active_threshold);
                Serial.print(") | Пик "); Serial.print(best_freq);
                Serial.print(" Гц: "); Serial.print(max_snr); Serial.println(" dB");
            } else {
                Serial.print("Фон адаптирован: "); Serial.print(dynamic_volume_floor);
                Serial.print(" | Текущий порог сработки: "); Serial.println(active_threshold);
            }
            last_debug_time = millis(); 
        }

        // 4. Временная валидация и защита от дребезга сигнала
        validator.update(frame_has_drone);
        bool drone_now_confirmed = validator.is_confirmed();

        static unsigned long last_time_drone_seen = 0;
        if (drone_now_confirmed) {
            last_time_drone_seen = millis();
        }

        // Удерживаем логическую тревогу в течение 5 секунд после того, как валидатор потерял сигнал
        bool should_be_alarm = drone_now_confirmed;
        if (!drone_now_confirmed && (millis() - last_time_drone_seen < 5000)) {
            if (drone_was_detected) {
                should_be_alarm = true; 
            }
        }

        // 5. Строгое частотное ограничение на вывод ALARM / CLEAR (не чаще раза в 10 сек)
        if (millis() - last_alarm_send_time >= ALARM_INTERVAL_MS) {
            if (should_be_alarm && !drone_was_detected) {
                Serial.println("\n=== ALARM ===");
                sendDirectMessage("ALARM", TARGET_NODE_ID);
                drone_was_detected = true;
                last_alarm_send_time = millis();
            } 
            else if (!should_be_alarm && drone_was_detected) {
                Serial.println("\n=== CLEAR ===");
                sendDirectMessage("CLEAR", TARGET_NODE_ID);
                drone_was_detected = false;
                last_alarm_send_time = millis();
            }
            else if (should_be_alarm && drone_was_detected) {
                // Если дрон продолжает находиться в зоне видимости, напоминаем раз в 10 секунд
                Serial.println("\n=== ALARM (STILL ACTIVE) ===");
                sendDirectMessage("ALARM", TARGET_NODE_ID);
                last_alarm_send_time = millis();
            }
        }
    }
}
