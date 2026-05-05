#pragma once

#include "../ld2402.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome::ld2402 {

class LD2402Sensor : public LD2402Listener, public Component, sensor::Sensor {
 public:
  void dump_config() override;
  void set_distance_sensor(sensor::Sensor *sensor) { this->distance_sensor_ = sensor; }
  void on_distance(uint16_t distance) override {
    if (this->distance_sensor_ != nullptr) {
      if (this->distance_sensor_->get_state() != distance) {
        this->distance_sensor_->publish_state(distance);
      }
    }
  }

 protected:
  sensor::Sensor *distance_sensor_{nullptr};
};

}  // namespace esphome::ld2402
