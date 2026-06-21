/* Includes ---------------------------------------------------------------- */
// Double quotes lagaye hain taaki error na aaye
#include "MSubhan-project-1_inferencing.h" 

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"

// OLED Display ke liye U8g2 Library
#include <Wire.h>
#include <U8g2lib.h>

// SH1106 OLED Display Setup (I2C Pins: SDA=21, SCL=22)
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

/** Audio buffers, pointers and selectors */
typedef struct {
    signed short *buffers[2];
    unsigned char buf_select;
    unsigned char buf_ready;
    unsigned int buf_count;
    unsigned int n_samples;
} inference_t;

static inference_t inference;
static const uint32_t sample_buffer_size = 2048;
static signed short sampleBuffer[sample_buffer_size];
static bool debug_nn = false; 
static int print_results = -(EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW);
static bool record_status = true;

// Custom Threshold - Kitne percent accuracy par action lena hai (0.8 = 80%)
const float CONFIDENCE_THRESHOLD = 0.8;

/**
 * @brief      Arduino setup function
 */
void setup()
{
    Serial.begin(115200);
    while (!Serial);
    
    // OLED Display Start Karna
    u8g2.begin();
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr); // Font set kiya
    u8g2.drawStr(10, 30, "System Starting...");
    u8g2.sendBuffer();

    Serial.println("Edge Impulse Inferencing Demo");

    run_classifier_init();
    
    if (microphone_inference_start(EI_CLASSIFIER_SLICE_SIZE) == false) {
        Serial.println("ERR: Could not allocate audio buffer");
        u8g2.clearBuffer();
        u8g2.drawStr(10, 30, "Mic Error!");
        u8g2.sendBuffer();
        return;
    }

    Serial.println("Recording...");
    u8g2.clearBuffer();
    u8g2.drawStr(20, 30, "Listening...");
    u8g2.sendBuffer();
}

/**
 * @brief      Arduino main function. Runs the inferencing loop.
 */
void loop()
{
    bool m = microphone_inference_record();
    if (!m) {
        Serial.println("ERR: Failed to record audio...");
        return;
    }

    signal_t signal;
    signal.total_length = EI_CLASSIFIER_SLICE_SIZE;
    signal.get_data = &microphone_audio_signal_get_data;
    ei_impulse_result_t result = {0};

    EI_IMPULSE_ERROR r = run_classifier_continuous(&signal, &result, debug_nn);
    if (r != EI_IMPULSE_OK) {
        Serial.printf("ERR: Failed to run classifier (%d)\n", r);
        return;
    }

    if (++print_results >= (EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW)) {
        
        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
            
            // "Hey Jarvis" command detect hone par
            if (strcmp(result.classification[ix].label, "Hey jarvis") == 0 && result.classification[ix].value > CONFIDENCE_THRESHOLD) {
                Serial.println("Command: Hey Jarvis -> Action: Hello Sir");
                
                u8g2.clearBuffer();
                u8g2.setFont(u8g2_font_ncenB10_tr); // Thoda bada font
                u8g2.drawStr(25, 35, "Hello Sir");
                u8g2.sendBuffer();
            }
            
            // "Jarvis there is a fire here" detect hone par
            else if (strcmp(result.classification[ix].label, "Jarvis there is a fire here") == 0 && result.classification[ix].value > CONFIDENCE_THRESHOLD) {
                Serial.println("Command: Fire Detected -> Action: Scanning Area");
                
                u8g2.clearBuffer();
                u8g2.setFont(u8g2_font_ncenB08_tr); // Normal font lambe text ke liye
                u8g2.drawStr(5, 25, "Ok sir,");
                u8g2.drawStr(5, 45, "I scan this area");
                u8g2.sendBuffer();
            }
        }
        print_results = 0;
    }
}

// ----------------------------------------------------------------------
// MICROPHONE AUR I2S KI SETTINGS NEECHE HAIN
// ----------------------------------------------------------------------

static void audio_inference_callback(uint32_t n_bytes)
{
    for(int i = 0; i < n_bytes>>1; i++) {
        inference.buffers[inference.buf_select][inference.buf_count++] = sampleBuffer[i];

        if(inference.buf_count >= inference.n_samples) {
            inference.buf_select ^= 1;
            inference.buf_count = 0;
            inference.buf_ready = 1;
        }
    }
}

static void capture_samples(void* arg) {
  const int32_t i2s_bytes_to_read = (uint32_t)arg;
  size_t bytes_read = i2s_bytes_to_read;

  while (record_status) {
    i2s_read((i2s_port_t)1, (void*)sampleBuffer, i2s_bytes_to_read, &bytes_read, 100);

    if (bytes_read > 0) {
        // Audio volume badhane ke liye (* 8)
        for (int x = 0; x < i2s_bytes_to_read/2; x++) {
            sampleBuffer[x] = (int16_t)(sampleBuffer[x]) * 8;
        }
        if (record_status) {
            audio_inference_callback(i2s_bytes_to_read);
        } else {
            break;
        }
    }
  }
  vTaskDelete(NULL);
}

static bool microphone_inference_start(uint32_t n_samples)
{
    inference.buffers[0] = (signed short *)malloc(n_samples * sizeof(signed short));
    if (inference.buffers[0] == NULL) return false;

    inference.buffers[1] = (signed short *)malloc(n_samples * sizeof(signed short));
    if (inference.buffers[1] == NULL) {
        ei_free(inference.buffers[0]);
        return false;
    }

    inference.buf_select = 0;
    inference.buf_count = 0;
    inference.n_samples = n_samples;
    inference.buf_ready = 0;

    if (i2s_init(EI_CLASSIFIER_FREQUENCY)) {
        Serial.println("Failed to start I2S!");
    }

    ei_sleep(100);
    record_status = true;
    xTaskCreate(capture_samples, "CaptureSamples", 1024 * 32, (void*)sample_buffer_size, 10, NULL);
    return true;
}

static bool microphone_inference_record(void)
{
    if (inference.buf_ready == 1) {
        Serial.println("Error sample buffer overrun.");
        return false;
    }
    while (inference.buf_ready == 0) {
        delay(1);
    }
    inference.buf_ready = 0;
    return true;
}

static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr)
{
    numpy::int16_to_float(&inference.buffers[inference.buf_select ^ 1][offset], out_ptr, length);
    return 0;
}

static void microphone_inference_end(void)
{
    i2s_deinit();
    ei_free(inference.buffers[0]);
    ei_free(inference.buffers[1]);
}

static int i2s_init(uint32_t sampling_rate) {
  i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
      .sample_rate = sampling_rate,
      .bits_per_sample = (i2s_bits_per_sample_t)16,
      .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT, // INMP441 default is L/R tied to GND -> RIGHT
      .communication_format = I2S_COMM_FORMAT_I2S,
      .intr_alloc_flags = 0,
      .dma_buf_count = 8,
      .dma_buf_len = 512,
      .use_apll = false,
      .tx_desc_auto_clear = false,
      .fixed_mclk = -1,
  };
  
  // AAPKE DIYE HUE MIC PINS YAHAN SET HAIN
  i2s_pin_config_t pin_config = {
      .bck_io_num = 26,    // I2S_MIC_SCK
      .ws_io_num = 25,     // I2S_MIC_WS
      .data_out_num = -1,  // Not used
      .data_in_num = 32,   // I2S_MIC_SD
  };
  esp_err_t ret = 0;

  ret = i2s_driver_install((i2s_port_t)1, &i2s_config, 0, NULL);
  if (ret != ESP_OK) Serial.println("Error in i2s_driver_install");

  ret = i2s_set_pin((i2s_port_t)1, &pin_config);
  if (ret != ESP_OK) Serial.println("Error in i2s_set_pin");

  ret = i2s_zero_dma_buffer((i2s_port_t)1);
  if (ret != ESP_OK) Serial.println("Error in initializing dma buffer with 0");

  return int(ret);
}

static int i2s_deinit(void) {
    i2s_driver_uninstall((i2s_port_t)1); 
    return 0;
}
