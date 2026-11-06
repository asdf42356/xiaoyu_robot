#pragma once
#include <driver/i2c_master.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define AHT10_I2C_ADDR 0x38
#define AHT10_SOFTRESET_CMD 0xBA
#define AHT10_INIT_CMD 0xE1
#define AHT10_TRIGGER_CMD 0xAC
#define AHT10_DATA_CMD 0x33
#define AHT10_DATA2_CMD 0x00

class AHT10 {
private:
    i2c_master_bus_handle_t i2c_bus_;
    i2c_master_dev_handle_t dev_handle_;
    bool initialized_;
    float temperature_;
    float humidity_;

    bool WriteCommand(uint8_t cmd, uint8_t data1 = 0, uint8_t data2 = 0) {
        uint8_t data[] = {cmd, data1, data2};
        esp_err_t ret = i2c_master_transmit(dev_handle_, data, sizeof(data), 100);
        return ret == ESP_OK;
    }

    bool WriteByte(uint8_t byte) {
        esp_err_t ret = i2c_master_transmit(dev_handle_, &byte, 1, 100);
        return ret == ESP_OK;
    }

    bool ReadData(uint8_t* data, size_t len) {
        esp_err_t ret = i2c_master_receive(dev_handle_, data, len, 100);
        return ret == ESP_OK;
    }

public:
    AHT10(i2c_master_bus_handle_t i2c_bus) : i2c_bus_(i2c_bus), initialized_(false), temperature_(0), humidity_(0) {}

    bool Init() {
        if (initialized_) return true;

        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = AHT10_I2C_ADDR,
            .scl_speed_hz = 100000
        };

        esp_err_t ret = i2c_master_bus_add_device(i2c_bus_, &dev_cfg, &dev_handle_);
        if (ret != ESP_OK) {
            ESP_LOGE("AHT10", "Failed to add device at 0x%02X: %s", AHT10_I2C_ADDR, esp_err_to_name(ret));
            return false;
        }

        // AHT10 出厂已校准，不需要 0xE1 初始化，等上电稳定即可
        vTaskDelay(pdMS_TO_TICKS(200));
        initialized_ = true;
        ESP_LOGI("AHT10", "Initialized at 0x%02X", AHT10_I2C_ADDR);
        return true;
    }

    bool ReadSensor() {
        if (!initialized_) return false;

        if (!WriteCommand(AHT10_TRIGGER_CMD, AHT10_DATA_CMD, AHT10_DATA2_CMD)) {
            return false;
        }

        vTaskDelay(pdMS_TO_TICKS(100));  // AHT10 测量最坏情况 80ms，给 100ms 余量

        uint8_t data[6] = {0};
        if (!ReadData(data, 6)) {
            return false;
        }

        if ((data[0] & 0x80) != 0) {
            ESP_LOGE("AHT10", "Sensor busy");
            return false;
        }

        uint32_t raw_humidity = ((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) | (data[3] >> 4);
        humidity_ = (float)raw_humidity * 100.0f / 1048576.0f;

        uint32_t raw_temperature = ((uint32_t)(data[3] & 0x0F) << 16) | ((uint32_t)data[4] << 8) | data[5];
        temperature_ = (float)raw_temperature * 200.0f / 1048576.0f - 53.0f;  // -3°C 补偿

        ESP_LOGI("AHT10", "Temp: %.1f°C, Humidity: %.1f%%", temperature_, humidity_);
        return true;
    }

    float GetTemperature() const { return temperature_; }
    float GetHumidity() const { return humidity_; }
};