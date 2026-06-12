#include "reconfig_buttons.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

static const char *const TAG = "ld2402.button";

namespace esphome::ld2402 {

void LD2402ApplyConfigButton::press_action() { this->parent_->apply_config_action(); }
void LD2402RevertConfigButton::press_action() { this->parent_->revert_config_action(); }
void LD2402FactoryResetButton::press_action() { this->parent_->factory_reset_action(); }
void LD2402AutoCalibrateButton::press_action() { this->parent_->auto_calibrate_action(); }
void LD2402SaveConfigButton::press_action() { this->parent_->save_config_action(); }

}  // namespace esphome::ld2402
