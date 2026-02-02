#include <RadioLib.h>
#include "esp32c6_hal.h"

static const char *TAG = "LoRa-C6";

// Adjust your actual pins here
#define PIN_SCK     19     // e.g. GPIO6
#define PIN_MISO    20    // e.g. GPIO7
#define PIN_MOSI    18    // e.g. GPIO8
#define PIN_CS      21      // e.g. GPIO10
#define PIN_RST     22     // e.g. GPIO3
#define PIN_DIO0    23    // e.g. GPIO4   (or DIO1 on SX126x)

EspHal_C6* hal = new EspHal_C6(PIN_SCK, PIN_MISO, PIN_MOSI, SPI2_HOST);

SX1276 radio = new Module(hal, PIN_CS, PIN_DIO0, PIN_RST, RADIOLIB_NC);  // DIO1 = NC if not used

// For SX1262 / SX1268 use: SX1262 radio = new Module(hal, PIN_CS, PIN_DIO1, PIN_RST, PIN_BUSY);

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "Initializing RadioLib on ESP32-C6...");

    hal->init();

    // Attach SPI device (important!)
    esp_err_t err = hal->attachDevice(PIN_CS, 4000000);  // 4 MHz is usually safe
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI device add failed: %d", err);
        while(1) vTaskDelay(1000);
    }

    int state = radio.begin();   // or begin(868.0, 125.0, 9) etc.
    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG, "Radio init failed: %d", state);
        while(1) vTaskDelay(1000);
    }

    ESP_LOGI(TAG, "Radio OK!");

    for(;;) {
        ESP_LOGI(TAG, "Transmitting...");
        state = radio.transmit("Hello from ESP32-C6!");
        if (state == RADIOLIB_ERR_NONE) {
            ESP_LOGI(TAG, "Sent OK");
        } else {
            ESP_LOGE(TAG, "TX failed: %d", state);
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

