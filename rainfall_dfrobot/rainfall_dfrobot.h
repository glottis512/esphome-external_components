#pragma once
#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
    namespace rainfall_dfrobot {
        class DFRobotRainfall : public PollingComponent, public i2c::I2CDevice {
        public:
            void set_rain_hour(uint8_t h) { this->rain_hour_ = h; }
            void set_total_rain_sensor(sensor::Sensor *s) { this->total_rain_mm = s; }
            void set_hour_rain_sensor(sensor::Sensor *s) { this->hour_rain_mm = s; }
            void set_raw_tips_sensor(sensor::Sensor *s) { this->raw_tips = s; }
            void set_working_hours_sensor(sensor::Sensor *s) { this->working_hours = s; }

            void set_base_rainfall_mm(float v) {
                this->base_rainfall_mm_ = v;
                this->set_base_pending_ = true;
            }

            void setup() override {
                uint8_t id[4] = {0};
                if (this->read_i2c_reg_(0x00, id, 4) == 4) {
                    uint32_t pid = (uint32_t) id[0] | ((uint32_t) id[1] << 8) | (((uint32_t)(id[3] & 0xC0)) << 10);
                    uint16_t vid = (uint16_t) id[2] | (uint16_t)((id[3] & 0x3F) << 8);
                    ESP_LOGI(TAG, "Detected PID=0x%05X, VID=0x%04X", (unsigned) pid, (unsigned) vid);
                }
            }

            void dump_config() override {
                LOG_I2C_DEVICE(this);
                ESP_LOGCONFIG(TAG, "DFRobot Rainfall (SEN0575)");
                ESP_LOGCONFIG(TAG, "  I2C address: 0x%02X", this->address_);
                ESP_LOGCONFIG(TAG, "  rain_hour: %u h", this->rain_hour_);
                ESP_LOGCONFIG(TAG, "  Sensors:");
                ESP_LOGCONFIG(TAG, "    total_rain_mm:   %s", this->total_rain_mm ? "configured" : "NOT configured");
                ESP_LOGCONFIG(TAG, "    hour_rain_mm:    %s", this->hour_rain_mm ? "configured" : "NOT configured");
                ESP_LOGCONFIG(TAG, "    raw_tips:        %s", this->raw_tips ? "configured" : "NOT configured");
                ESP_LOGCONFIG(TAG, "    working_hours:   %s", this->working_hours ? "configured" : "NOT configured");
            }

            float get_setup_priority() const override { return setup_priority::DATA; }

            void update() override {
                if (this->set_base_pending_) {
                    uint16_t base_raw = (uint16_t)(this->base_rainfall_mm_ * 10000.0f);
                    uint8_t buf[2] = {(uint8_t)(base_raw & 0xFF), (uint8_t)(base_raw >> 8)};
                    if (this->write_i2c_reg_(0x28, buf, 2) != 0) {
                        ESP_LOGW(TAG, "Failed to set BASE_RAINFALL");
                    } else {
                        this->set_base_pending_ = false;
                    }
                    delay(120);
                }

                uint8_t cr[4] = {0};
                if (this->read_i2c_reg_(0x10, cr, 4) == 4) {
                    ESP_LOGV(TAG, "CUM raw: %02X %02X %02X %02X", cr[0], cr[1], cr[2], cr[3]);
                    uint32_t raw = (uint32_t) cr[0] | ((uint32_t) cr[1] << 8) | ((uint32_t) cr[2] << 16) | ((uint32_t) cr[3] << 24);
                    float mm = (float) raw / 10000.0f;
                    if (this->total_rain_mm) this->total_rain_mm->publish_state(mm);
                }

                if (this->rain_hour_ >= 1 && this->rain_hour_ <= 24) {
                    uint8_t hr = this->rain_hour_;
                    this->write_i2c_reg_(0x26, &hr, 1);
                    delay(20);
                    uint8_t tr[4] = {0};
                    if (this->read_i2c_reg_(0x0C, tr, 4) == 4) {
                        ESP_LOGV(TAG, "TIME(%u h) raw: %02X %02X %02X %02X", this->rain_hour_, tr[0], tr[1], tr[2], tr[3]);
                        uint32_t raw = (uint32_t) tr[0] | ((uint32_t) tr[1] << 8) | ((uint32_t) tr[2] << 16) | ((uint32_t) tr[3] << 24);
                        float mm = (float) raw / 10000.0f;
                        if (this->hour_rain_mm) this->hour_rain_mm->publish_state(mm);
                    }
                }

                uint8_t rd[4] = {0};
                if (this->read_i2c_reg_(0x14, rd, 4) == 4) {
                    ESP_LOGV(TAG, "RAW tips: %02X %02X %02X %02X", rd[0], rd[1], rd[2], rd[3]);
                    uint32_t raw = (uint32_t) rd[0] | ((uint32_t) rd[1] << 8) | ((uint32_t) rd[2] << 16) | ((uint32_t) rd[3] << 24);
                    if (this->raw_tips) this->raw_tips->publish_state((float) raw);
                }

                uint8_t wt[2] = {0};
                if (this->read_i2c_reg_(0x18, wt, 2) == 2) {
                    ESP_LOGV(TAG, "WT raw: %02X %02X", wt[0], wt[1]);
                    uint16_t mins = (uint16_t) wt[0] | ((uint16_t) wt[1] << 8);
                    float hours = (float) mins / 60.0f;
                    if (this->working_hours) this->working_hours->publish_state(hours);
                }
            }

        protected:
            int read_i2c_reg_(uint8_t reg, uint8_t *buf, size_t len) {
                if (!buf || len == 0) return 0;
                for (int attempt = 0; attempt < 3; ++attempt) {
                    auto rc = this->write_read(&reg, 1, buf, len);
                    if (rc == i2c::ERROR_OK || rc == i2c::NO_ERROR) return (int) len;
                    delay(2);
                }
                return 0;
            }

            int write_i2c_reg_(uint8_t reg, const uint8_t *buf, size_t len) {
                if (!buf || len == 0) return -1;
                uint8_t tmp[16];
                if (len + 1 <= sizeof(tmp)) {
                    tmp[0] = reg;
                    for (size_t i = 0; i < len; i++) tmp[1 + i] = buf[i];
                    auto rc = this->write(tmp, len + 1);
                    return (rc == i2c::ERROR_OK || rc == i2c::NO_ERROR) ? 0 : -1;
                }
                auto r1 = this->write(&reg, 1);
                if (!(r1 == i2c::ERROR_OK || r1 == i2c::NO_ERROR)) return -1;
                auto r2 = this->write(buf, len);
                return (r2 == i2c::ERROR_OK || r2 == i2c::NO_ERROR) ? 0 : -1;
            }

            uint8_t rain_hour_ = 1;
            bool set_base_pending_ = false;
            float base_rainfall_mm_ = 0.2794f;
            sensor::Sensor *total_rain_mm{nullptr};
            sensor::Sensor *hour_rain_mm{nullptr};
            sensor::Sensor *raw_tips{nullptr};
            sensor::Sensor *working_hours{nullptr};

            static constexpr const char *TAG = "rainfall_dfrobot";
        };
    } // namespace rainfall_dfrobot
} // namespace esphome
