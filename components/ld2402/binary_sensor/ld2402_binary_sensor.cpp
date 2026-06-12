#include "ld2402_binary_sensor.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome::ld2402 {

static const char *const TAG = "ld2402.binary_sensor";

void LD2402BinarySensor::dump_config() {
  ESP_LOGCONFIG(TAG, "Binary Sensor:");
  LOG_BINARY_SENSOR("  ", "Presence", this->presence_bsensor_);
}

}  // namespace esphome::ld2402
