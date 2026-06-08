#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/automation.h"
#include "esphome/core/helpers.h"
#include <span>
#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif
#ifdef USE_NUMBER
#include "esphome/components/number/number.h"
#endif
#ifdef USE_BUTTON
#include "esphome/components/button/button.h"
#endif

namespace esphome::ld2402 {

// UART line buffer limits: longest frame (141 B) plus resync/search slack.
static constexpr uint8_t MAX_LINE_LENGTH = 160;
static constexpr uint8_t TOTAL_GATES = 16;

class LD2402Listener {
 public:
  virtual void on_presence(bool presence){};
  virtual void on_moving_target(bool moving_target){};
  virtual void on_still_target(bool still_target){};
  virtual void on_distance(uint16_t distance){};
  virtual void on_fw_version(std::string &fw){};
};

class LD2402Component : public Component, public uart::UARTDevice {
 public:
  struct CmdFrameT {
    uint32_t header{0};
    uint32_t footer{0};
    uint16_t length{0};
    uint16_t command{0};
    uint16_t data_length{0};
    uint8_t data[18];
  };

  struct RegConfigT {
    uint32_t move_thresh[TOTAL_GATES];
    uint32_t still_thresh[TOTAL_GATES];
    uint16_t max_gate{0};
    uint16_t timeout{0};
  };

  void setup() override;
  void dump_config() override;
  void loop() override;
#ifdef USE_NUMBER
  void set_gate_timeout_number(number::Number *number) { this->gate_timeout_number_ = number; };
  void set_gate_select_number(number::Number *number) { this->gate_select_number_ = number; };
  void set_max_gate_distance_number(number::Number *number) { this->max_gate_distance_number_ = number; };
  void set_gate_move_sensitivity_factor_number(number::Number *number) {
    this->gate_move_sensitivity_factor_number_ = number;
  };
  void set_gate_still_sensitivity_factor_number(number::Number *number) {
    this->gate_still_sensitivity_factor_number_ = number;
  };
  void set_gate_still_threshold_numbers(int gate, number::Number *n) { this->gate_still_threshold_numbers_[gate] = n; };
  void set_gate_move_threshold_numbers(int gate, number::Number *n) { this->gate_move_threshold_numbers_[gate] = n; };
  bool is_gate_select() { return gate_select_number_ != nullptr; };
  uint8_t get_gate_select_value() { return static_cast<uint8_t>(this->gate_select_number_->state); };
  void publish_gate_move_threshold(uint8_t gate) {
    // With gate_select we only use 1 number pointer, thus we hard code [0]
    this->gate_move_threshold_numbers_[0]->publish_state(this->new_config.move_thresh[gate]);
  };
  void publish_gate_still_threshold(uint8_t gate) {
    this->gate_still_threshold_numbers_[0]->publish_state(this->new_config.still_thresh[gate]);
  };
  void init_gate_config_numbers();
  void refresh_gate_config_numbers();
#endif
#ifdef USE_BUTTON
  void set_apply_config_button(button::Button *button) { this->apply_config_button_ = button; };
  void set_revert_config_button(button::Button *button) { this->revert_config_button_ = button; };
  void set_restart_module_button(button::Button *button) { this->restart_module_button_ = button; };
  void set_factory_reset_button(button::Button *button) { this->factory_reset_button_ = button; };
#endif
  void register_listener(LD2402Listener *listener) { this->listeners_.push_back(listener); }

  void send_module_restart();
  void restart_module_action();
  void apply_config_action();
  void factory_reset_action();
  void revert_config_action();
  float get_setup_priority() const override;
  int send_cmd_from_array(CmdFrameT cmd_frame);
  void handle_cmd_error(uint8_t error);
  uint8_t set_config_mode(bool enable);
  void set_system_mode(uint16_t mode);
  void set_max_distance_and_timeout(uint32_t max_gate_distance, uint32_t timeout);
  void set_gate_threshold(uint8_t gate);
  void ld2402_restart();

  float gate_move_sensitivity_factor{0.5};
  float gate_still_sensitivity_factor{0.5};
  int32_t last_periodic_millis{0};
  RegConfigT current_config;
  RegConfigT new_config;
#ifdef USE_BUTTON
  button::Button *apply_config_button_{nullptr};
  button::Button *revert_config_button_{nullptr};
  button::Button *restart_module_button_{nullptr};
  button::Button *factory_reset_button_{nullptr};
#endif

 protected:
  struct CmdReplyT {
    uint32_t data[4];
    uint16_t error;
    uint8_t command;
    uint8_t status;
    uint8_t length;
    volatile bool ack;
  };

  void get_firmware_version_();
  int get_gate_threshold_(uint8_t gate);
  int get_max_distance_and_timeout_();
  uint16_t get_mode_() { return this->system_mode_; };
  void set_mode_(uint16_t mode) { this->system_mode_ = mode; };
  bool get_presence_() { return this->presence_; };
  bool get_moving_target_() { return this->moving_target_; };
  bool get_still_target_() { return this->still_target_; };
  void set_presence_(bool presence) { this->presence_ = presence; };
  void set_moving_target_(bool moving_target) { this->moving_target_ = moving_target; };
  void set_still_target_(bool still_target) { this->still_target_ = still_target; };
  uint16_t get_distance_() { return this->distance_; };
  void set_distance_(uint16_t distance) { this->distance_ = distance; };
  void handle_detection_frame_(uint8_t *buffer, int len);
  void handle_ack_data_(uint8_t *buffer, int len);
  void readline_(int rx_data, uint8_t *buffer, int len, uint8_t &buffer_pos);
  void read_batch_(std::span<uint8_t, MAX_LINE_LENGTH> buffer);
  void append_rx_byte_(int rx_data, uint8_t *buffer, int len, uint8_t &buffer_pos);
  void resync_buffer_(uint8_t *buffer, uint8_t &buffer_pos);
  bool validate_detection_frame_(const uint8_t *buffer, int len) const;
  bool validate_cmd_frame_(const uint8_t *buffer, int len) const;

#ifdef USE_NUMBER
  number::Number *gate_timeout_number_{nullptr};
  number::Number *gate_select_number_{nullptr};
  number::Number *max_gate_distance_number_{nullptr};
  number::Number *gate_move_sensitivity_factor_number_{nullptr};
  number::Number *gate_still_sensitivity_factor_number_{nullptr};
  std::vector<number::Number *> gate_still_threshold_numbers_ = std::vector<number::Number *>(16);
  std::vector<number::Number *> gate_move_threshold_numbers_ = std::vector<number::Number *>(16);
#endif

  uint16_t distance_{0};
  uint16_t system_mode_;
  uint8_t detection_buffer_pos_{0};
  uint8_t cmd_buffer_pos_{0};
  uint8_t buffer_data_[MAX_LINE_LENGTH];
  char firmware_ver_[8]{"v0.0.0"};
  bool cmd_active_{false};
  bool presence_{false};
  bool moving_target_{false};
  bool still_target_{false};
  CmdReplyT cmd_reply_;
  std::vector<LD2402Listener *> listeners_{};
};

}  // namespace esphome::ld2402
