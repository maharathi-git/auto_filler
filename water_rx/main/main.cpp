#include <RadioLib.h>
#include "esp32c6_hal.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "LoRa-RX-Status";

// ───────────────────────────────────────────────
//          STATUS LED PINS (single RGB or separate LEDs)
// ───────────────────────────────────────────────
// Assuming separate LEDs or RGB common cathode
#define LED_GREEN   GPIO_NUM_5     // Green = success / packet received
#define LED_RED     GPIO_NUM_4     // Red   = timeout / no packet

#define LED_ON      0              // Common cathode: LOW = ON
#define LED_OFF     1              // HIGH = OFF

#define STATUS_GLOW_TIME_MS   2000  // How long to show color

// ───────────────────────────────────────────────
//          RADIO PINS (your previous / adjust if needed)
// ───────────────────────────────────────────────
#define PIN_SCK     19
#define PIN_MISO    20
#define PIN_MOSI    18
#define PIN_CS      21
#define PIN_RST     22
#define PIN_DIO0    23           // DIO0 for SX127x packet done interrupt

EspHal_C6* hal = new EspHal_C6(PIN_SCK, PIN_MISO, PIN_MOSI, SPI2_HOST);

// SX1276 example (change to SX1262 if your module is newer)
SX1276 radio = new Module(hal, PIN_CS, PIN_DIO0, PIN_RST, RADIOLIB_NC);
// If SX126x: SX1262 radio = new Module(hal, PIN_CS, PIN_DIO1, PIN_RST, PIN_BUSY);

void setStatusGreen() {
    gpio_set_level(LED_GREEN, LED_ON);
    gpio_set_level(LED_RED,   LED_OFF);
}

void setStatusRed() {
    gpio_set_level(LED_RED,   LED_ON);
    gpio_set_level(LED_GREEN, LED_OFF);
}

void clearStatus() {
    gpio_set_level(LED_RED,   LED_OFF);
    gpio_set_level(LED_GREEN, LED_OFF);
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "Starting LoRa Receiver with LED status on ESP32-C6...");

    // LED pins setup
    gpio_set_direction(LED_GREEN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_RED,   GPIO_MODE_OUTPUT);
    clearStatus();

    // HAL & SPI init
    hal->init();
    esp_err_t err = hal->attachDevice(PIN_CS, 4000000);  // 4 MHz SPI
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI device attach failed: %d", err);
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Radio begin (default LoRa params, adjust freq/BW/SF/CR if needed)
    int state = radio.begin();
    // Example custom: radio.begin(433.0, 125.0, 9, 5, SX127X_SYNC_WORD, 17, 10, 0);
    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG, "Radio init failed: %d", state);
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGI(TAG, "Radio ready. Waiting for packets...");

    for(;;) {
        char buffer[65] = {0};  // Buffer for received data + null

        // Receive with timeout (0 = auto based on LoRa config, or set explicit ms)
        state = radio.receive((uint8_t*)buffer, 64, 5000);  // 5 sec timeout example

        if (state == RADIOLIB_ERR_NONE) {
            // Success: packet received
            size_t len = radio.getPacketLength();
            buffer[len] = '\0';  // Safe string

            float rssi = radio.getRSSI();
            float snr  = radio.getSNR();

            ESP_LOGI(TAG, "PACKET OK | Data: '%s' | RSSI: %.1f dBm | SNR: %.1f dB", 
                     buffer, rssi, snr);

            setStatusGreen();
            vTaskDelay(pdMS_TO_TICKS(STATUS_GLOW_TIME_MS));
            clearStatus();
        }
        else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
            // Timeout: no packet in time
            float last_rssi = radio.getRSSI();  // Last measured RSSI (even on timeout)
            float last_snr  = radio.getSNR();

            ESP_LOGW(TAG, "RX TIMEOUT | Last RSSI: %.1f dBm | Last SNR: %.1f dB", 
                     last_rssi, last_snr);

            setStatusRed();
            vTaskDelay(pdMS_TO_TICKS(STATUS_GLOW_TIME_MS));
            clearStatus();
        }
        else {
            // Other error
            ESP_LOGE(TAG, "Receive error: %d", state);
            // Optional: short red blink for error
            setStatusRed();
            vTaskDelay(pdMS_TO_TICKS(300));
            clearStatus();
        }

        // Small loop delay to avoid 100% CPU
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
