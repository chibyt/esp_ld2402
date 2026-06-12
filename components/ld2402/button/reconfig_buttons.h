#pragma once

#include "esphome/components/button/button.h"
#include "../ld2402.h"

namespace esphome::ld2402 {

class LD2402ApplyConfigButton : public button::Button, public Parented<LD2402Component> {
 public:
  LD2402ApplyConfigButton() = default;

 protected:
  void press_action() override;
};

class LD2402FactoryResetButton : public button::Button, public Parented<LD2402Component> {
 public:
  LD2402FactoryResetButton() = default;

 protected:
  void press_action() override;
};

class LD2402AutoCalibrateButton : public button::Button, public Parented<LD2402Component> {
 public:
  LD2402AutoCalibrateButton() = default;

 protected:
  void press_action() override;
};

}  // namespace esphome::ld2402
