// #include <Arduino.h>
// #include <driver/i2s_std.h>
// #include <math.h>

// #define I2S_BCLK_PIN GPIO_NUM_41
// #define I2S_WS_PIN   GPIO_NUM_42
// #define I2S_DATA_PIN GPIO_NUM_40

// #define SAMPLE_RATE     16000  
// #define BLOCK_SIZE      1024   

// // --- ПЛОТНАЯ СЕТКА ЧАСТОТ ---
// // Добавляем частоты с шагом 5-10 Гц, чтобы реальный гул БПЛА гарантированно попал в фильтр
// const float DRONE_FREQS[] = {
//     140.0f, 145.0f, 150.0f, 155.0f, 160.0f, 165.0f, 170.0f, 175.0f, 180.0f, 185.0f, 190.0f, 195.0f,
//     200.0f, 205.0f, 210.0f, 215.0f, 220.0f, 225.0f, 230.0f, 235.0f, 240.0f, 245.0f, 250.0f
// }; 
// const int NUM_FREQS = sizeof(DRONE_FREQS) / sizeof(DRONE_FREQS[0]); // ИСПРАВЛЕНО ОКОНЧАТЕЛЬНО

// // --- НАСТРОЙКИ ФИЛЬТРАЦИИ ---
// const float SNR_THRESHOLD_DB = 7.5f;  // Опускаем до 7.5 дБ, так как реальный дрон создает "размазанный" пик
// const float MIN_VOLUME_RMS = 180.0f;  // Порог шумоподавления для тишины

// i2s_chan_handle_t rx_handle = NULL;
// int16_t audio_buffer[BLOCK_SIZE];
// bool drone_was_detected = false;

// // --- ИНТЕГРАЛЬНЫЙ АДАПТИВНЫЙ ФИЛЬТР ---
// // Вместо жесткого окна плавно накапливает энергию тревоги
// int alarm_accumulator = 0;
// const int ALARM_THRESHOLD = 25;       // Порог включения тревоги (накопленный)
// const int ALARM_MAX = 40;             // Потолок накопителя

// void init_i2s() {
//     i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
//     i2s_new_channel(&chan_cfg, NULL, &rx_handle);

//     i2s_std_config_t std_cfg = {
//         .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
//         .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
//         .gpio_cfg = {
//             .mclk = I2S_GPIO_UNUSED,
//             .bclk = I2S_BCLK_PIN,
//             .ws = I2S_WS_PIN,
//             .dout = I2S_GPIO_UNUSED,
//             .din = I2S_DATA_PIN,
//             .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false }
//         }
//     };
    
//     std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT; 
//     i2s_channel_init_std_mode(rx_handle, &std_cfg);
//     i2s_channel_enable(rx_handle);
// }

// float goertzel_energy(const int16_t* samples, int num_samples, float target_freq, float sampling_rate) {
//     float k = 0.5f + ((float)num_samples * target_freq) / sampling_rate;
//     float omega = (2.0f * M_PI / (float)num_samples) * k;
//     float cosine = cosf(omega);
//     float coefficient = 2.0f * cosine;

//     float q0 = 0.0f, q1 = 0.0f, q2 = 0.0f;

//     for (int i = 0; i < num_samples; i++) {
//         float x = (float)samples[i] / 32768.0f; 
//         q0 = x + coefficient * q1 - q2;
//         q2 = q1;
//         q1 = q0;
//     }
//     return (q1 * q1) + (q2 * q2) - (coefficient * q1 * q2);
// }

// // Измененная функция: замеряет шум чуть дальше (на 35 Гц), чтобы не глушить плотную сетку частот
// bool check_peak(const int16_t* samples, int num_samples, float target_freq, float snr_threshold) {
//     float target_energy = goertzel_energy(samples, num_samples, target_freq, SAMPLE_RATE);
//     float noise_l = goertzel_energy(samples, num_samples, target_freq - 35.0f, SAMPLE_RATE);
//     float noise_r = goertzel_energy(samples, num_samples, target_freq + 35.0f, SAMPLE_RATE);
//     float avg_noise = (noise_l + noise_r) / 2.0f;

//     if (avg_noise <= 0.0f) avg_noise = 0.00001f;

//     float snr_db = 10.0f * log10f(target_energy / avg_noise);
//     return (snr_db > snr_threshold);
// }

// float calculate_rms(const int16_t* samples, int num_samples) {
//     double sum = 0;
//     for (int i = 0; i < num_samples; i++) {
//         sum += (double)samples[i] * samples[i];
//     }
//     return sqrt(sum / num_samples);
// }

// void setup() {
//     Serial.begin(115200);
//     init_i2s();
//     Serial.println("Система готова к тесту реального звука БПЛА...");
// }

// void loop() {
//     size_t bytes_read = 0;
//     esp_err_t res = i2s_channel_read(rx_handle, audio_buffer, sizeof(audio_buffer), &bytes_read, portMAX_DELAY);
    
//     if (res == ESP_OK && bytes_read > 0) {
//         bool frame_has_drone = false;
//         float current_rms = calculate_rms(audio_buffer, BLOCK_SIZE);

//         if (current_rms > MIN_VOLUME_RMS) {
//             for (int i = 0; i < NUM_FREQS; i++) {
//                 if (check_peak(audio_buffer, BLOCK_SIZE, DRONE_FREQS[i], SNR_THRESHOLD_DB)) {
//                     frame_has_drone = true;
//                     break; 
//                 }
//             }
//         }

//         // --- ИНТЕГРАЛЬНАЯ ЛОГИКА НАКОПЛЕНИЯ ---
//         if (frame_has_drone) {
//             alarm_accumulator += 4; // Быстро добавляем очки при совпадении
//             if (alarm_accumulator > ALARM_MAX) alarm_accumulator = ALARM_MAX;
//         } else {
//             alarm_accumulator -= 1; // Медленно сбрасываем при пропусках (сглаживает хрипы динамика)
//             if (alarm_accumulator < 0) alarm_accumulator = 0;
//         }

//         // Переключение триггера тревоги
//         if (alarm_accumulator >= ALARM_THRESHOLD && !drone_was_detected) {
//             Serial.println("ALARM");
//             drone_was_detected = true;
//         } 
//         else if (alarm_accumulator == 0 && drone_was_detected) {
//             Serial.println("CLEAR");
//             drone_was_detected = false;
//         }
//     }
//     delay(2); // Минимальная задержка для плотного потока вычислений
// }






#include <Arduino.h>
#include <driver/i2s_std.h>
#include <math.h>

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

const float SNR_THRESHOLD_DB = 7.0f;  

// --- НАСТРОЙКИ ДИНАМИЧЕСКОГО ШУМОПОДАВЛЕНИЯ ---
float dynamic_volume_floor = 150.0f;  // Начальное значение уровня фонового шума
const float NOISE_MARGIN = 80.0f;     // Запас (гистерезис) над фоном. Порог сработки = фон + этот запас
const float ADAPTATION_RATE = 0.005f; // Скорость адаптации к фону (чем меньше число, тем плавнее подстройка)

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
    init_i2s();
    Serial.println("Система с адаптивным шумоподавлением запущена...");
}

unsigned long last_debug_time = 0;

void loop() {
    size_t bytes_read = 0;
    esp_err_t res = i2s_channel_read(rx_handle, audio_buffer, sizeof(audio_buffer), &bytes_read, portMAX_DELAY);
    
    if (res == ESP_OK && bytes_read > 0) {
        bool frame_has_drone = false;
        float current_rms = calculate_rms(audio_buffer, BLOCK_SIZE);
        
        // Текущий порог сработки рассчитывается динамически на ходу
        float active_threshold = dynamic_volume_floor + NOISE_MARGIN;

        bool show_debug = (millis() - last_debug_time > 500);

        if (current_rms > active_threshold) {
            float max_snr = -99.0f;
            float best_freq = 0;

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

            if (show_debug) {
                Serial.print("Громкость: "); Serial.print(current_rms);
                Serial.print(" (Порог: "); Serial.print(active_threshold);
                Serial.print(") | Пик "); Serial.print(best_freq);
                Serial.print(" Гц: "); Serial.print(max_snr); Serial.println(" dB");
                last_debug_time = millis();
            }
        } 
        else {
            // Если звук тихий и нет тревоги, плавно подстраиваем "пол шума" под текущую обстановку
            if (!drone_was_detected) {
                dynamic_volume_floor = (dynamic_volume_floor * (1.0f - ADAPTATION_RATE)) + (current_rms * ADAPTATION_RATE);
            }

            if (show_debug) {
                Serial.print("Фон адаптирован: "); Serial.print(dynamic_volume_floor);
                Serial.print(" | Текущий порог сработки: "); Serial.println(active_threshold);
                last_debug_time = millis();
            }
        }

        validator.update(frame_has_drone);
        bool drone_now_confirmed = validator.is_confirmed();

        if (drone_now_confirmed && !drone_was_detected) {
            Serial.println("\n=== ALARM ===");
            drone_was_detected = true;
        } 
        else if (!drone_now_confirmed && drone_was_detected) {
            Serial.println("\n=== CLEAR ===");
            drone_was_detected = false;
        }
    }
    delay(2); 
}
