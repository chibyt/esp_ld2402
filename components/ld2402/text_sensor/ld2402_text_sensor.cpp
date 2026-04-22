#include "ld2402_text_sensor.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome::ld2402 {

static const char *const TAG = "ld2402.text_sensor";

void LD2402TextSensor::dump_config() {
  ESP_LOGCONFIG(TAG, "Text Sensor:");
  LOG_TEXT_SENSOR("  ", "Firmware", this->fw_version_text_sensor_);
}

}  // namespace esphome::ld2402
