#include <Arduino.h>
#include "driver/i2s.h"
#include "esp_dsp.h"

#define I2S_BCLK_PIN GPIO_NUM_41
#define I2S_WS_PIN   GPIO_NUM_42
#define I2S_DATA_PIN GPIO_NUM_40
#define I2S_PORT I2S_NUM_0 

#define SAMPLE_RATE     16000
#define FFT_SIZE        1024
#define BIN_RESOLUTION  ((float)SAMPLE_RATE / FFT_SIZE) 

#define F0_MIN_HZ       180.0f
#define F0_MAX_HZ       700.0f  
#define F0_STEP_HZ      5.0f   

// Пороги чувствительности оптимизированы под слабую 3-ю гармонику
#define SNR_THRESHOLD   4.5f    
#define NOISE_ALPHA     0.02f   
#define MIN_NOISE_FLOOR 0.0003f 

// Интеграционный гистерезис: мягкий накопитель баллов
#define ALARM_SCORE_MAX   12    
#define ALARM_SCORE_THRES 4     // Нужно набрать всего 4 балла (2 удачных кадра) для ALARM

int32_t i2s_raw_buffer[FFT_SIZE];
__attribute__((aligned(16))) float fft_input[FFT_SIZE * 2];
__attribute__((aligned(16))) float dsp_window[FFT_SIZE];
float magnitude[FFT_SIZE / 2];
float noise_floor[FFT_SIZE / 2];

bool is_alarm = false;
int alarm_score = 0; 
float tracked_f0 = 0.0f; // Сопровождаемая частота БПЛА

float goertzel_magnitude(int32_t* num_array, int samples_count, float target_freq, float sampling_rate) {
    float k = 0.5f + ((float)samples_count * target_freq) / sampling_rate;
    float omega = (2.0f * PI * k) / (float)samples_count;
    float sine = sinf(omega);
    float cosine = cosf(omega);
    float coeff = 2.0f * cosine;

    float q0 = 0, q1 = 0, q2 = 0;

    for (int i = 0; i < samples_count; i++) {
        float sample = (float)(num_array[i] >> 8) / 8388608.0f; 
        q0 = coeff * q1 - q2 + sample;
        q2 = q1;
        q1 = q0;
    }

    float power = (q1 * q1 + q2 * q2 - q1 * q2 * coeff);
    return sqrtf(power) * (2.0f / (float)samples_count);
}

float get_noise_for_freq(float freq) {
    int bin = (int)(freq / BIN_RESOLUTION);
    if (bin >= FFT_SIZE / 2) bin = (FFT_SIZE / 2) - 1;
    if (bin < 0) bin = 0;
    return (noise_floor[bin] < MIN_NOISE_FLOOR) ? MIN_NOISE_FLOOR : noise_floor[bin];
}

void init_i2s() {
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 4,
        .dma_buf_len = 256,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_BCLK_PIN,
        .ws_io_num = I2S_WS_PIN,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_DATA_PIN
    };

    i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_PORT, &pin_config);
    i2s_zero_dma_buffer(I2S_PORT);
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("[INIT] BATEAR Final Rock-Solid UAV Detector Starting...");

    if (dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE) != ESP_OK) {
        Serial.println("[ERROR] DSP FFT Init Failed");
        while (1);
    }
    dsps_wind_hann_f32(dsp_window, FFT_SIZE);
    init_i2s();

    for (int i = 0; i < FFT_SIZE / 2; i++) {
        noise_floor[i] = MIN_NOISE_FLOOR; 
    }
    Serial.println("[INIT] Tracking system initialized.");
}

void loop() {
    size_t bytes_read = 0;
    esp_err_t res = i2s_read(I2S_PORT, i2s_raw_buffer, sizeof(i2s_raw_buffer), &bytes_read, portMAX_DELAY);
    
    if (res != ESP_OK || bytes_read == 0) return;

    unsigned long start_time = micros();

    // 1. Спектральный анализ (FFT)
    for (int i = 0; i < FFT_SIZE; i++) {
        float sample = (float)(i2s_raw_buffer[i] >> 8) / 8388608.0f; 
        fft_input[i * 2 + 0] = sample * dsp_window[i];
        fft_input[i * 2 + 1] = 0.0f;
    }

    dsps_fft2r_fc32(fft_input, FFT_SIZE);
    dsps_bit_rev_fc32(fft_input, FFT_SIZE);

    float current_avg_noise = 0.0f;
    for (int i = 0; i < FFT_SIZE / 2; i++) {
        float real = fft_input[i * 2 + 0];
        float imag = fft_input[i * 2 + 1];
        magnitude[i] = sqrtf(real * real + imag * imag) * (2.0f / FFT_SIZE);

        if (magnitude[i] < noise_floor[i] * SNR_THRESHOLD) {
            noise_floor[i] = (NOISE_ALPHA * magnitude[i]) + ((1.0f - NOISE_ALPHA) * noise_floor[i]);
        } else {
            noise_floor[i] = ((NOISE_ALPHA * 0.05f) * magnitude[i]) + ((1.0f - (NOISE_ALPHA * 0.05f)) * noise_floor[i]);
        }
        if (noise_floor[i] < MIN_NOISE_FLOOR) noise_floor[i] = MIN_NOISE_FLOOR;
        
        if (i >= 12 && i < 40) current_avg_noise += noise_floor[i];
    }
    current_avg_noise /= 28.0f;

    // 2. Валидация гармоник Герцелем + Фильтр спектральной чистоты
    bool drone_detected_this_frame = false;
    float best_f0 = 0.0f;
    float max_total_snr = 0.0f;
    float d_snr_f0 = 0, d_snr_f1 = 0, d_snr_f2 = 0;

    float safe_f0_min = 220.0f; 

    for (float f0 = safe_f0_min; f0 <= F0_MAX_HZ; f0 += F0_STEP_HZ) {
        if (f0 >= 355.0f && f0 <= 365.0f) continue; // Сетевой фильтр 360 Гц

        float f1 = f0 * 2.0f;
        float f2 = f0 * 3.0f;
        float f_mid = f0 * 1.5f; 

        float amp_f0       = goertzel_magnitude(i2s_raw_buffer, FFT_SIZE, f0, SAMPLE_RATE);
        float amp_f0_left  = goertzel_magnitude(i2s_raw_buffer, FFT_SIZE, f0 - 15.0f, SAMPLE_RATE);
        float amp_f0_right = goertzel_magnitude(i2s_raw_buffer, FFT_SIZE, f0 + 15.0f, SAMPLE_RATE);
        
        float amp_f1  = goertzel_magnitude(i2s_raw_buffer, FFT_SIZE, f1, SAMPLE_RATE);
        float amp_f2  = goertzel_magnitude(i2s_raw_buffer, FFT_SIZE, f2, SAMPLE_RATE);
        float amp_mid = goertzel_magnitude(i2s_raw_buffer, FFT_SIZE, f_mid, SAMPLE_RATE);

        float noise_f0  = get_noise_for_freq(f0);
        float noise_f1  = get_noise_for_freq(f1);
        float noise_f2  = get_noise_for_freq(f2);
        float noise_mid = get_noise_for_freq(f_mid);

        float snr_f0  = amp_f0 / noise_f0;
        float snr_f1  = amp_f1 / noise_f1;
        float snr_f2  = amp_f2 / noise_f2;
        float snr_mid = amp_mid / noise_mid;

        // Порог детекции адаптивен к общему шуму ветра
        float base_threshold = (f0 < 280.0f) ? 5.8f : 3.4f;
        float current_threshold = (current_avg_noise > 0.00045f) ? (base_threshold + 2.5f) : base_threshold;

        // Метрика чистоты пика (Анти-Ветер)
        if (amp_f0_left > (amp_f0 * 0.65f) || amp_f0_right > (amp_f0 * 0.65f)) continue;

        // Фильтр хлопков / резких шумов
        if (snr_mid > (snr_f0 * 0.60f)) continue; 

        // --- МЕТРИКА БАЛАНСА ЭНЕРГИИ ЛОПАСТЕЙ (БЛОКИРОВКА ВЕТРА) ---
        // У реального дрона на частотах выше 400 Гц вторая гармоника (f1) обязана быть сильной.
        // Если f0 превышает f1 более чем в 4.5 раза (как в вашем логе ветра 77.0 / 7.4 = 10.4), это 100% удар ветра!
        if (f0 > 400.0f && (snr_f0 / (snr_f1 + 1e-6f) > 4.5f)) continue;

        // Фильтр аномальных наводок
        if (snr_f0 > 80.0f && snr_f2 < 6.0f) continue;

        // Основное спектральное сито
        if (snr_f0 > current_threshold && snr_f1 > (current_threshold * 0.75f) && snr_f2 > 1.2f) {
            float current_total_snr = snr_f0 + snr_f1 + snr_f2;
            if (current_total_snr > max_total_snr) {
                max_total_snr = current_total_snr;
                best_f0 = f0;
                d_snr_f0 = snr_f0;
                d_snr_f1 = snr_f1;
                d_snr_f2 = snr_f2;
                drone_detected_this_frame = true;
            }
        }
    }

    static int track_lifetime = 0; 

    // 3. Интеграционный трекинг частоты (Ужесточенный коридор против ветра)
    if (drone_detected_this_frame) {
        if (tracked_f0 == 0.0f) {
            tracked_f0 = best_f0;
            track_lifetime = 15; // Сократили удержание до 15 кадров (~1 сек)
            alarm_score += 2;
        } 
        // СУЗИЛИ КОРИДОР ДО ±20 Гц: скачки ветра вроде 540 -> 515 (дельта 25) теперь разрушают трек
        else if (abs(best_f0 - tracked_f0) <= 20.0f) {
            alarm_score += 2;
            track_lifetime = 15; 
            tracked_f0 = (0.25f * best_f0) + (0.75f * tracked_f0); 
        } 
        else {
            alarm_score -= 2; // Жесткий штраф за смену частоты
        }
    } else {
        alarm_score -= 1; 
        if (tracked_f0 > 0.0f) {
            track_lifetime--;
            if (track_lifetime <= 0) tracked_f0 = 0.0f; 
        }
    }

    // Защита от выхода Score за границы [0, ALARM_SCORE_MAX]
    if (alarm_score < 0) alarm_score = 0;
    if (alarm_score > ALARM_SCORE_MAX) alarm_score = ALARM_SCORE_MAX;

    // УВЕЛИЧИЛИ ПОРОГ ВЗВОДА ДО 6 (требуется минимум 3 стабильных кадра подряд)
    int active_alarm_threshold = 6;

    // Триггеры тревоги
    if (!is_alarm && (alarm_score >= active_alarm_threshold)) {
        is_alarm = true;
        Serial.println("\n##################################################");
        Serial.printf("## ALARM: UAV DETECTED! Long-Track F0: %.1f Hz ##\n", tracked_f0);
        Serial.println("##################################################\n");
    } 
    else if (is_alarm && (alarm_score == 0)) {
        is_alarm = false;
        Serial.println("\n--------------------------------------------------");
        Serial.println("-- CLEAR: UAV Target Disappeared --");
        Serial.println("--------------------------------------------------\n");
    }

    unsigned long processing_time = micros() - start_time;

    // 4. Логирование
    static uint32_t last_debug_ms = 0;
    if (millis() - last_debug_ms > 1000 || drone_detected_this_frame) {
        last_debug_ms = millis();

        Serial.printf("[DEBUG] Time: %lu us | State: %s | Match: %s | Score: %d/%d", 
                      processing_time, 
                      is_alarm ? "ALARM" : "IDLE", 
                      drone_detected_this_frame ? "YES" : "NO",
                      alarm_score, active_alarm_threshold);

        if (drone_detected_this_frame) {
            Serial.printf(" | F0: %.1f Hz (Track: %.1f, Life: %d) | Total SNR: %.2f (f0:%.1f, f1:%.1f, f2:%.1f)", 
                          best_f0, tracked_f0, track_lifetime, max_total_snr, d_snr_f0, d_snr_f1, d_snr_f2);
        } else {
            Serial.printf(" | Base Noise: %.6f | Track: %.1f", current_avg_noise, tracked_f0);
        }
        Serial.println();
    }
}
