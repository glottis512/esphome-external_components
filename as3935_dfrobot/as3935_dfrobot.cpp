#include "as3935_dfrobot.h"

namespace esphome {
namespace as3935_dfrobot {

static const char *const TAG = "as3935_dfrobot";

void AS3935DFRobot::setup() {
  ESP_LOGCONFIG(TAG, "Setting up AS3935 (DFRobot/I2C, soft-IRQ on GPIO)…");

  if (this->irq_pin_ != nullptr) {
    this->irq_pin_->setup();
    this->last_irq_level_ = this->irq_pin_->digital_read();
    this->last_irq_valid_ = true;
  }

  this->reset_();
  this->write_masked(REG_AFE_GB, 0x01, 0x00);
  this->calibrate_rco_();
  this->write_masked(REG_MISC, 0x20, 0x20);
  delay(2);
  this->write_masked(REG_MISC, 0x20, 0x00);
  this->configure_();

  if (this->tune_antenna_enabled_) {
    this->tune_antenna_();
  }

  (void)this->read_interrupt_source_();
}

void AS3935DFRobot::dump_config() {
  ESP_LOGCONFIG(TAG, "AS3935 DFRobot (I2C, soft-IRQ)");
  LOG_I2C_DEVICE(this);
  ESP_LOGCONFIG(TAG, "  IRQ pin: %s", this->irq_pin_ ? "configured" : "(none)");
  ESP_LOGCONFIG(TAG, "  Mode: %s", indoor_ ? "INDOOR" : "OUTDOOR");
  ESP_LOGCONFIG(TAG, "  Mask disturbers: %s", mask_dist_ ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Noise level (NF): %u", noise_level_);
  ESP_LOGCONFIG(TAG, "  Watchdog threshold (WDTH): %u", watchdog_threshold_);
  ESP_LOGCONFIG(TAG, "  Spike rejection (SREJ): %u", spike_rejection_);
  ESP_LOGCONFIG(TAG, "  Lightning threshold (min strikes): %u", lightning_threshold_);
  ESP_LOGCONFIG(TAG, "  Tuning cap (0..15): %u", tuning_cap_);
}

void AS3935DFRobot::update() {
  bool triggered = false;
  if (this->irq_pin_ != nullptr) {
    bool level = this->irq_pin_->digital_read();
    if (this->last_irq_valid_ && level && !this->last_irq_level_) {
      uint32_t now = millis();
      if (now - this->last_irq_ms_ > 2)
        triggered = true;
      this->last_irq_ms_ = now;
    }
    this->last_irq_level_ = level;
    this->last_irq_valid_ = true;
  }

  if (!triggered && this->irq_pin_ == nullptr) {
    triggered = (this->read_interrupt_source_() != 0);
    if (!triggered) return;
  }

  uint8_t src = this->read_interrupt_source_();
  if (src == 0)
    return;

  if (src == 1) {
    uint8_t dist = this->read_distance_km_();
    uint32_t energy = this->read_energy_raw_();
    if (distance_sensor_ != nullptr)
      distance_sensor_->publish_state(dist);
    if (energy_sensor_ != nullptr)
      energy_sensor_->publish_state(energy);
    if (event_text_sensor_ != nullptr)
      event_text_sensor_->publish_state("LIGHTNING");
  } else if (src == 2) {
    if (event_text_sensor_ != nullptr)
      event_text_sensor_->publish_state("DISTURBER");
  } else if (src == 3) {
    if (event_text_sensor_ != nullptr)
      event_text_sensor_->publish_state("NOISE");
  }
}

i2c::ErrorCode AS3935DFRobot::write_register(uint8_t reg, uint8_t value) {
  uint8_t buf[2] = {reg, value};
  auto err = this->write(buf, sizeof(buf));
  if (err != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "i2c write_register(reg=0x%02X) err=%d", reg, (int)err);
  }
  return err;
}

i2c::ErrorCode AS3935DFRobot::read_register(uint8_t reg, uint8_t &value) {
  auto err = this->write_read(&reg, 1, &value, 1);
  if (err == i2c::ERROR_OK) return err;
  ESP_LOGV(TAG, "i2c write_read(reg=0x%02X) err=%d, trying STOP+START", reg, (int)err);
  err = this->write(&reg, 1);
  if (err != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "i2c write(reg=0x%02X) err=%d", reg, (int)err);
    return err;
  }
  delayMicroseconds(50);
  err = this->read(&value, 1);
  if (err != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "i2c read(reg=0x%02X) err=%d", reg, (int)err);
  }
  return err;
}

i2c::ErrorCode AS3935DFRobot::write_masked(uint8_t reg, uint8_t mask, uint8_t value) {
  uint8_t cur = 0;
  auto err = this->read_register(reg, cur);
  if (err != i2c::ERROR_OK) return err;
  uint8_t nw = (cur & ~mask) | (value & mask);
  if (nw == cur) return i2c::ERROR_OK;
  return this->write_register(reg, nw);
}

void AS3935DFRobot::reset_() {
  this->write_register(CMD_PRESET_DEFAULT, 0x96);
  delay(2);
}

void AS3935DFRobot::calibrate_rco_() {
  this->write_register(CMD_CALIB_RCO, 0x96);
  delay(2);
}

void AS3935DFRobot::configure_() {
  this->write_masked(REG_AFE_GB, 0x3E, indoor_ ? 0x24 : 0x1C);
  this->write_masked(REG_INT_LCO, 0x20, mask_dist_ ? 0x20 : 0x00);
  this->write_masked(REG_NF_WDTH, 0x70, (uint8_t)((noise_level_ & 0x07) << 4));
  this->write_masked(REG_NF_WDTH, 0x0F, (uint8_t)(watchdog_threshold_ & 0x0F));
  this->write_masked(REG_SREJ_MISC, 0x0F, (uint8_t)(spike_rejection_ & 0x0F));
  uint8_t min_bits = 0x00;
  if (lightning_threshold_ >= 16)
    min_bits = 0x30;
  else if (lightning_threshold_ >= 9)
    min_bits = 0x20;
  else if (lightning_threshold_ >= 5)
    min_bits = 0x10;
  else
    min_bits = 0x00;
  this->write_masked(REG_SREJ_MISC, 0x30, min_bits);
  this->write_masked(REG_MISC, 0x0F, (uint8_t)(tuning_cap_ & 0x0F));
}

static inline uint16_t fdiv_to_divisor(uint16_t fdiv) {
  switch (fdiv) {
    case 16: return 16;
    case 32: return 32;
    case 64: return 64;
    case 128: return 128;
    default: return 16;
  }
}

void AS3935DFRobot::tune_antenna_() {
  if (this->irq_pin_ == nullptr) {
    ESP_LOGW(TAG, "tune_antenna requested but no IRQ pin configured; skipping");
    return;
  }
  ESP_LOGI(TAG, "Starting antenna tuning (target %u kHz, fdiv=%u)", (unsigned)lco_target_khz_, (unsigned)lco_fdiv_);
  this->write_masked(REG_MISC, 0xE0, 0x80);
  uint8_t fdiv_bits = 0x00;
  if (lco_fdiv_ == 32) fdiv_bits = 0x40;
  else if (lco_fdiv_ == 64) fdiv_bits = 0x80;
  else if (lco_fdiv_ == 128) fdiv_bits = 0xC0;
  this->write_masked(REG_INT_LCO, 0xC0, fdiv_bits);
  delay(10);
  uint32_t best_err = 0xFFFFFFFFUL;
  uint8_t best_cap = tuning_cap_ & 0x0F;
  const uint32_t window_ms = 100;
  const uint32_t target_div = (uint32_t) lco_target_khz_ * 1000UL / fdiv_to_divisor(lco_fdiv_);
  for (uint8_t cap = 0; cap < 16; cap++) {
    this->write_masked(REG_MISC, 0x0F, cap);
    delay(5);
    uint32_t meas_hz = this->measure_lco_hz_((uint8_t)lco_fdiv_, window_ms);
    if (meas_hz == 0) continue;
    uint32_t err = (meas_hz > target_div ? meas_hz - target_div : target_div - meas_hz);
    ESP_LOGI(TAG, "cap=%u -> LCO/%u ~ %u Hz (err=%u)", cap, (unsigned)lco_fdiv_, (unsigned)meas_hz, (unsigned)err);
    if (err < best_err) { best_err = err; best_cap = cap; }
  }
  this->write_masked(REG_MISC, 0x0F, best_cap);
  this->write_masked(REG_MISC, 0xE0, 0x00);
  tuning_cap_ = best_cap;
  ESP_LOGI(TAG, "Antenna tuned: best tuning_cap=%u", best_cap);
}

uint32_t AS3935DFRobot::measure_lco_hz_(uint8_t, uint32_t window_ms) {
  bool prev = this->irq_pin_->digital_read();
  uint32_t edges = 0;
  uint32_t start = millis();
  while ((millis() - start) < window_ms) {
    bool cur = this->irq_pin_->digital_read();
    if (cur && !prev) {
      edges++;
    }
    prev = cur;
    delayMicroseconds(5);
  }
  uint32_t freq = (edges * 1000UL) / (window_ms == 0 ? 1 : window_ms);
  return freq;
}

uint8_t AS3935DFRobot::read_interrupt_source_() {
  delay(10);
  uint8_t v1 = 0;
  if (this->read_register(REG_INT_LCO, v1) != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "read INT (0x03) failed");
    return 0;
  }
  uint8_t code = (v1 & 0x0F);
  if (code == 0x08) return 1;
  if (code == 0x04) return 2;
  if (code == 0x01) return 3;
  if (this->irq_pin_ != nullptr && this->irq_pin_->digital_read()) {
    delay(2);
    uint8_t v2 = 0;
    if (this->read_register(REG_INT_LCO, v2) == i2c::ERROR_OK) {
      uint8_t code2 = (v2 & 0x0F);
      if (code2 == 0x08) return 1;
      if (code2 == 0x04) return 2;
      if (code2 == 0x01) return 3;
    }
  }
  ESP_LOGV(TAG, "INT raw=0x%02X (low nibble=0x%02X)", v1, (v1 & 0x0F));
  return 0;
}

uint8_t AS3935DFRobot::read_distance_km_() {
  uint8_t v = 0;
  if (this->read_register(REG_DISTANCE, v) != i2c::ERROR_OK) return 0x3F;
  return (v & 0x3F);
}

uint32_t AS3935DFRobot::read_energy_raw_() {
  uint8_t e_l = 0, e_m = 0, e_mm = 0;
  this->read_register(REG_E_L, e_l);
  this->read_register(REG_E_M, e_m);
  this->read_register(REG_E_MM, e_mm);
  uint32_t energy = ((uint32_t)(e_mm & 0x1F) << 16) | ((uint32_t)e_m << 8) | e_l;
  return energy / 16777U;
}

}  // namespace as3935_dfrobot
}  // namespace esphome