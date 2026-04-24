#pragma once

#include "../ld2402.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

namespace esphome::ld2402 {
class LD2402BinarySensor : public LD2402Listener, public Component {
//class LD2402BinarySensor : public LD2402Listener, public Component, binary_sensor::BinarySensor {
 public:
  void dump_config() override;
  void set_presence_sensor(binary_sensor::BinarySensor *bsensor) { this->presence_bsensor_ = bsensor; };
  void set_moving_target_sensor(binary_sensor::BinarySensor *bsensor) { this->moving_target_bsensor_ = bsensor; };
  void set_still_target_sensor(binary_sensor::BinarySensor *bsensor) { this->still_target_bsensor_ = bsensor; };
  void on_presence(bool presence) override {
    if (this->presence_bsensor_ != nullptr) {
      if (this->presence_bsensor_->state != presence)
        this->presence_bsensor_->publish_state(presence);
    }
  }
  void on_moving_target(bool moving_target) override {
    if (this->moving_target_bsensor_ != nullptr) {
      if (this->moving_target_bsensor_->state != moving_target)
        this->moving_target_bsensor_->publish_state(moving_target);
    }
  }
  void on_still_target(bool still_target) override {
    if (this->still_target_bsensor_ != nullptr) {
      if (this->still_target_bsensor_->state != still_target)
        this->still_target_bsensor_->publish_state(still_target);
    }
  }

 protected:
  binary_sensor::BinarySensor *presence_bsensor_{nullptr};
  binary_sensor::BinarySensor *moving_target_bsensor_{nullptr};
  binary_sensor::BinarySensor *still_target_bsensor_{nullptr};
};

}  // namespace esphome::ld2402
