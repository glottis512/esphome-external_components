#pragma once
#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

namespace esphome {
namespace as3935_dfrobot {

static const uint8_t REG_AFE_GB     = 0x00;
static const uint8_t REG_NF_WDTH    = 0x01;
static const uint8_t REG_SREJ_MISC  = 0x02;
static const uint8_t REG_INT_LCO    = 0x03;
static const uint8_t REG_E_L        = 0x04;
static const uint8_t REG_E_M        = 0x05;
static const uint8_t REG_E_MM       = 0x06;
static const uint8_t REG_DISTANCE   = 0x07;
static const uint8_t REG_MISC       = 0x08;

static const uint8_t CMD_PRESET_DEFAULT = 0x3C;
static const uint8_t CMD_CALIB_RCO     = 0x3D;

class AS3935DFRobot : public PollingComponent, public i2c::I2CDevice {
 public:
  void set_indoor(bool v) { indoor_ = v; }
  void set_mask_disturbers(bool v) { mask_dist_ = v; }
  void set_noise_level(uint8_t v) { noise_level_ = v; }
  void set_spike_rejection(uint8_t v) { spike_rejection_ = v; }
  void set_watchdog_threshold(uint8_t v) { watchdog_threshold_ = v; }
  void set_lightning_threshold(uint8_t v) { lightning_threshold_ = v; }
  void set_tuning_cap(uint8_t v) { tuning_cap_ = v & 0x0F; }
  void set_tune_antenna(bool v) { tune_antenna_enabled_ = v; }
  void set_lco_target_khz(uint16_t v) { lco_target_khz_ = v; }
  void set_lco_fdiv(uint16_t v) { lco_fdiv_ = v; }
  void set_irq_pin(GPIOPin *pin) { irq_pin_ = pin; }
  void set_distance_sensor(sensor::Sensor *s) { distance_sensor_ = s; }
  void set_energy_sensor(sensor::Sensor *s) { energy_sensor_ = s; }
  void set_event_text_sensor(text_sensor::TextSensor *s) { event_text_sensor_ = s; }

  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

 protected:
  void update() override;
  i2c::ErrorCode write_register(uint8_t reg, uint8_t value);
  i2c::ErrorCode read_register(uint8_t reg, uint8_t &value);
  i2c::ErrorCode write_masked(uint8_t reg, uint8_t mask, uint8_t value);
  void reset_();
  void calibrate_rco_();
  void configure_();
  void tune_antenna_();
  uint32_t measure_lco_hz_(uint8_t fdiv_sel, uint32_t window_ms);
  uint8_t read_interrupt_source_();
  uint8_t read_distance_km_();
  uint32_t read_energy_raw_();

  bool indoor_{false};
  bool mask_dist_{false};
  uint8_t noise_level_{2};
  uint8_t spike_rejection_{2};
  uint8_t watchdog_threshold_{2};
  uint8_t lightning_threshold_{1};
  uint8_t tuning_cap_{12};
  bool tune_antenna_enabled_{false};
  uint16_t lco_target_khz_{500};
  uint16_t lco_fdiv_{16};

  GPIOPin *irq_pin_{nullptr};
  bool last_irq_level_{false};
  bool last_irq_valid_{false};
  uint32_t last_irq_ms_{0};

  sensor::Sensor *distance_sensor_{nullptr};
  sensor::Sensor *energy_sensor_{nullptr};
  text_sensor::TextSensor *event_text_sensor_{nullptr};
};

}  // namespace as3935_dfrobot
}  // namespace esphome