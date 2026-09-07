#include "driver/i2s_std.h"

#define I2S_BK_IO      GPIO_NUM_41  // SCK / BCLK
#define I2S_WS_IO      GPIO_NUM_42  // WS / LRCLK
#define I2S_DO_IO      GPIO_NUM_40  // SD / DOUT

#define SAMPLE_RATE    16000        // Частота дискретизации (16 кГц)
#define BUFFER_SIZE    512          // Размер буфера в сэмплах

i2s_chan_handle_t rx_handle;

void init_i2s() {
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    i2s_new_channel(&chan_cfg, NULL, &rx_handle);

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        // ICS-43434 выдает 24 бита в 32-битном слоте
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BK_IO,
            .ws   = I2S_WS_IO,
            .dout = I2S_GPIO_UNUSED,
            .din  = I2S_DO_IO,
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
        },
    };

    std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT; // Смените на RIGHT, если LR на 3V3

    i2s_channel_init_std_mode(rx_handle, &std_cfg);
    i2s_channel_enable(rx_handle);
}

void setup() {
    // Высокая скорость порта, чтобы успевать передавать поток данных без задержек
    Serial.begin(500000); 
    while (!Serial) { delay(10); }
    
    init_i2s();
}

void loop() {
    int32_t i2s_buffer[BUFFER_SIZE];
    int16_t pcm_buffer[BUFFER_SIZE]; // Буфер для отправки на ПК (16-бит)
    size_t bytes_read = 0;

    // Читаем 32-битные данные из I2S
    esp_err_t result = i2s_channel_read(rx_handle, &i2s_buffer, sizeof(i2s_buffer), &bytes_read, portMAX_DELAY);

    if (result == ESP_OK && bytes_read > 0) {
        int samples_read = bytes_read / sizeof(int32_t);
        
        for (int i = 0; i < samples_read; i++) {
            // Восстанавливаем знак (выравниваем 24-битное число)
            int32_t raw_sample = i2s_buffer[i] >> 8;
            
            // Сжимаем 24 бита до стандартных 16 бит для WAV-файла на ПК.
            // Микрофон ICS-43434 очень чувствительный, сдвиг на 4-7 бит уберет лишний шум
            pcm_buffer[i] = (int16_t)(raw_sample >> 5); 
        }

        // Отправляем сырые байты 16-битного звука в Serial порт
        Serial.write((uint8_t*)pcm_buffer, samples_read * sizeof(int16_t));
    }
}
