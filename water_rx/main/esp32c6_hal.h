// EspHal_C6.h
#ifndef ESP_HAL_C6_H
#define ESP_HAL_C6_H

#include <RadioLib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "soc/spi_periph.h"           // for SPI signals
#include "soc/gpio_sig_map.h"         // for GPIO matrix signals on C6

#define LOW     0
#define HIGH    1
#define INPUT   GPIO_MODE_INPUT
#define OUTPUT  GPIO_MODE_OUTPUT
#define RISING  GPIO_INTR_POSEDGE
#define FALLING GPIO_INTR_NEGEDGE

// No need for old DPORT / rtc / spi_reg stuff

class EspHal_C6 : public RadioLibHal {
public:
    EspHal_C6(int8_t sck, int8_t miso, int8_t mosi, spi_host_device_t spi_host = SPI2_HOST)
        : RadioLibHal(INPUT, OUTPUT, LOW, HIGH, RISING, FALLING),
          _sck(sck), _miso(miso), _mosi(mosi), _spi_host(spi_host), _spi(NULL) {}

    void init() override {
        spi_bus_config_t buscfg = {};
        buscfg.miso_io_num   = _miso;
        buscfg.mosi_io_num   = _mosi;
        buscfg.sclk_io_num   = _sck;
        buscfg.quadwp_io_num = -1;
        buscfg.quadhd_io_num = -1;
        buscfg.max_transfer_sz = 4096;

        esp_err_t err = spi_bus_initialize(_spi_host, &buscfg, SPI_DMA_CH_AUTO);
        if (err != ESP_OK) {
            ESP_LOGE("EspHal_C6", "SPI bus init failed: %d", err);
        }
    }

    void term() override {
        if (_spi) {
            spi_bus_remove_device(_spi);
            _spi = NULL;
        }
        spi_bus_free(_spi_host);
    }

    // GPIO methods (same as before, very portable)
    void pinMode(uint32_t pin, uint32_t mode) override {
        if (pin == RADIOLIB_NC) return;
        gpio_config_t conf = {
            .pin_bit_mask = (1ULL << pin),
            .mode         = (gpio_mode_t)mode,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        gpio_config(&conf);
    }

    void digitalWrite(uint32_t pin, uint32_t val) override {
        if (pin == RADIOLIB_NC) return;
        gpio_set_level((gpio_num_t)pin, val);
    }

    uint32_t digitalRead(uint32_t pin) override {
        if (pin == RADIOLIB_NC) return 0;
        return gpio_get_level((gpio_num_t)pin);
    }

    void attachInterrupt(uint32_t pin, void (*cb)(void), uint32_t mode) override {
        if (pin == RADIOLIB_NC) return;
        gpio_install_isr_service(0);
        gpio_set_intr_type((gpio_num_t)pin, (gpio_int_type_t)mode);
        gpio_isr_handler_add((gpio_num_t)pin, (gpio_isr_t)cb, NULL);
    }

    void detachInterrupt(uint32_t pin) override {
        if (pin == RADIOLIB_NC) return;
        gpio_isr_handler_remove((gpio_num_t)pin);
        gpio_set_intr_type((gpio_num_t)pin, GPIO_INTR_DISABLE);
    }

    void delay(unsigned long ms) override { vTaskDelay(pdMS_TO_TICKS(ms)); }
    void delayMicroseconds(unsigned long us) override { esp_rom_delay_us(us); }
    unsigned long millis() override { return (unsigned long)(esp_timer_get_time() / 1000ULL); }
    unsigned long micros() override { return (unsigned long)esp_timer_get_time(); }

        // ─── SPI using spi_master ───────────────────────────────────────────────
    void spiBegin() override {
        // already done in init()
    }

    void spiBeginTransaction() override {
        // spi_master handles per-transaction config
    }


    void spiTransfer(uint8_t* out, size_t len, uint8_t* in) override {
        if (!_spi || len == 0) return;

        spi_transaction_t t = {};
        t.length    = len * 8;
        t.tx_buffer = out;
        t.rx_buffer = in;

        esp_err_t err = spi_device_transmit(_spi, &t);
        if (err != ESP_OK) {
            ESP_LOGE("EspHal_C6", "spiTransfer bulk failed: %d", err);
        }
    }

    void spiEndTransaction() override {}
    void spiEnd() override { /* handled in term() */ }

    // Call this after creating the radio object to add the device
    esp_err_t attachDevice(uint8_t cs_pin, uint32_t freq_hz = 2000000) {
        spi_device_interface_config_t devcfg = {};
        devcfg.command_bits     = 0;
        devcfg.address_bits     = 0;
        devcfg.mode             = 0;                // SPI mode 0 (most LoRa modules)
        devcfg.clock_speed_hz   = freq_hz;
        devcfg.spics_io_num     = cs_pin;
        devcfg.queue_size       = 1;
        devcfg.flags            = SPI_DEVICE_HALFDUPLEX;

        return spi_bus_add_device(_spi_host, &devcfg, &_spi);
    }

    long pulseIn(uint32_t pin, uint32_t state, RadioLibTime_t timeout) override {
    if (pin == RADIOLIB_NC) return 0;

    uint64_t start = micros();
    uint64_t now   = start;

    // Wait for pin to become == state
    while (digitalRead(pin) != state) {
        now = micros();
        if ((now - start) > (uint64_t)timeout) {
            return 0;
        }
    }

    start = now;

    // Now wait for pin to become != state
    while (digitalRead(pin) == state) {
        now = micros();
        if ((now - start) > (uint64_t)timeout) {
            return 0;
        }
    }

    return (long)(now - start);
   }
private:
    int8_t _sck, _miso, _mosi;
    spi_host_device_t _spi_host;
    spi_device_handle_t _spi = NULL;
};

#endif

