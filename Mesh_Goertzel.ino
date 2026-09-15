#include <Arduino.h>
#include <driver/i2s_std.h>
#include <math.h>

#include <pb_encode.h>
#include <pb_decode.h>

// Автоматически определяем правильный путь к структурам Meshtastic
#if __has_include(<meshtastic/mesh.pb.h>)
  #include <meshtastic/mesh.pb.h>
#else
  #include <mesh.pb.h>
#endif

// Настройка пинов UART для ESP32
#define ESP_RX_PIN 16 
#define ESP_TX_PIN 17 

HardwareSerial EspSerial(2);

// ID целевой ноды (c1e3b1c4)
const uint32_t TARGET_NODE_ID = 0xA7EEF537; 

// Настройка пинов для микрофона
#define I2S_BCLK_PIN GPIO_NUM_41
#define I2S_WS_PIN   GPIO_NUM_42
#define I2S_DATA_PIN GPIO_NUM_40

#define SAMPLE_RATE     16000  
#define BLOCK_SIZE      1024   

// Сетка частот БПЛА
const float DRONE_FREQS[] = {
    140.0f, 150.0f, 160.0f, 170.0f, 180.0f, 190.0f, 
    200.0f, 210.0f, 220.0f, 230.0f, 240.0f, 250.0f
}; 
const int NUM_FREQS = 12; 

const float SNR_THRESHOLD_DB = 3.0f;  

// --- НАСТРОЙКИ ДИНАМИЧЕСКОГО ШУМОПОДАВЛЕНИЯ ---
float dynamic_volume_floor = 150.0f;  
const float NOISE_MARGIN = 80.0f;     
const float ADAPTATION_RATE = 0.005f; 

// --- УСИЛЕННАЯ ВАЛИДАЦИЯ VO VREMENI ---
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

// --- ТАЙМЕРЫ ЗАЩИТЫ КАНАЛА СВЯЗИ ---
unsigned long last_alarm_time = 0;
unsigned long last_drone_seen_time = 0; 
const unsigned long ALARM_COOLDOWN_MS = 10000; // Жесткое ограничение ALARM (не чаще раза в 10 сек)
const unsigned long CLEAR_HOLD_MS = 7000;      // Задержка перед снятием тревоги увеличена до 7 секунд

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

float goertzel_energy(const int16_t* samples, int num_samples, float target_freq, float sampling_rate) {
    float k = 0.5f + ((float)num_samples * target_freq) / sampling_rate;
    float omega = (2.0f * M_PI / (float)num_samples) * k;
    float cosine = cosf(omega);
    float coefficient = 2.0f * cosine;

    float q0 = 0.0f, q1 = 0.0f, q2 = 0.0f;

    for (int i = 0; i < num_samples; i++) {
        float x = (float)samples[i] / 32768.0f; 
        q0 = x + coefficient * q1 - q2;
        q2 = q1;
        q1 = q0;
    }
    return (q1 * q1) + (q2 * q2) - (coefficient * q1 * q2);
}

float get_snr(const int16_t* samples, int num_samples, float target_freq) {
    float target_energy = goertzel_energy(samples, num_samples, target_freq, SAMPLE_RATE);
    float noise_l = goertzel_energy(samples, num_samples, target_freq - 25.0f, SAMPLE_RATE);
    float noise_r = goertzel_energy(samples, num_samples, target_freq + 25.0f, SAMPLE_RATE);
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

void setup() {
    Serial.begin(115200);

    // В бинарном режиме PROTO Meshtastic работает строго на 115200 бод
    EspSerial.begin(115200, SERIAL_8N1, ESP_RX_PIN, ESP_TX_PIN);
    delay(1000);

    // Синхронизируем Serial-интерфейс стартовыми wake-байтами
    for(int i = 0; i < 4; i++) {
    EspSerial.write(0x94);
    }

    Serial.println("ESP32 Мастер (Встроенный API Meshtastic/Nanopb) запущен.");

    init_i2s();
    Serial.println("Система детекции БПЛА запущена в бесшумном режиме.");
    last_alarm_time = millis();
    last_drone_seen_time = millis();
}

void loop() {
    size_t bytes_read = 0;
    esp_err_t res = i2s_channel_read(rx_handle, audio_buffer, sizeof(audio_buffer), &bytes_read, portMAX_DELAY);
    
    if (res == ESP_OK && bytes_read > 0) {
        bool frame_has_drone = false;
        float current_rms = calculate_rms(audio_buffer, BLOCK_SIZE);
        float active_threshold = dynamic_volume_floor + NOISE_MARGIN;

        if (current_rms > active_threshold) {
            for (int i = 0; i < NUM_FREQS; i++) {
                float snr = get_snr(audio_buffer, BLOCK_SIZE, DRONE_FREQS[i]);
                if (snr > SNR_THRESHOLD_DB) {
                    frame_has_drone = true;
                    break; 
                }
            }
        } 
        
        if (!frame_has_drone) {
            dynamic_volume_floor = (dynamic_volume_floor * (1.0f - ADAPTATION_RATE)) + (current_rms * ADAPTATION_RATE);
        }

        validator.update(frame_has_drone);
        bool drone_now_confirmed = validator.is_confirmed();

        // --- ЛОГИКА ТРЕВОГИ С ЖЕСТКИМ ТАЙМАУТОМ ---
        unsigned long current_time = millis();

        if (drone_now_confirmed) {
            // ЖЕСТКИЙ ЛИМИТ: Между отправками ALARM ОБЯЗАНЫ пройти 10 секунд (10000 мс)
            if (current_time - last_alarm_time >= ALARM_COOLDOWN_MS) {
                Serial.println("=== ALARM ===");
                sendDirectMessage("ALARM", TARGET_NODE_ID);
                
                // МЕСТО ДЛЯ КОДА ОТПРАВКИ (ESP-NOW, LoRa, Wi-Fi)
                
                last_alarm_time = current_time;
            }
            drone_was_detected = true;
            last_drone_seen_time = current_time; // Обновляем метку времени, когда дрон был "виден"
        } 
        else {
            if (drone_was_detected) {
                // Снимаем статус только после 7 секунд полной тишины
                if (current_time - last_drone_seen_time >= CLEAR_HOLD_MS) {
                    Serial.println("=== CLEAR ===");
                    sendDirectMessage("CLEAR", TARGET_NODE_ID);
                    drone_was_detected = false;
                }
            }
        }
    }
}
