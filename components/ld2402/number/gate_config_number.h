#pragma once

#include "esphome/components/number/number.h"
#include "../ld2402.h"

namespace esphome::ld2402 {

class LD2402TimeoutNumber : public number::Number, public Parented<LD2402Component> {
 public:
  LD2402TimeoutNumber() = default;

 protected:
  void control(float timeout) override;
};

class LD2402MinDistanceNumber : public number::Number, public Parented<LD2402Component> {
 public:
  LD2402MinDistanceNumber() = default;

 protected:
  void control(float min_gate) override;
};

class LD2402MaxDistanceNumber : public number::Number, public Parented<LD2402Component> {
 public:
  LD2402MaxDistanceNumber() = default;

 protected:
  void control(float max_gate) override;
};

class LD2402GateSelectNumber : public number::Number, public Parented<LD2402Component> {
 public:
  LD2402GateSelectNumber() = default;

 protected:
  void control(float gate_select) override;
};

class LD2402MoveSensFactorNumber : public number::Number, public Parented<LD2402Component> {
 public:
  LD2402MoveSensFactorNumber() = default;

 protected:
  void control(float move_factor) override;
};

class LD2402StillSensFactorNumber : public number::Number, public Parented<LD2402Component> {
 public:
  LD2402StillSensFactorNumber() = default;

 protected:
  void control(float still_factor) override;
};

class LD2402StillThresholdNumbers : public number::Number, public Parented<LD2402Component> {
 public:
  LD2402StillThresholdNumbers() = default;
  LD2402StillThresholdNumbers(uint8_t gate);

 protected:
  uint8_t gate_;
  void control(float still_threshold) override;
};

class LD2402MoveThresholdNumbers : public number::Number, public Parented<LD2402Component> {
 public:
  LD2402MoveThresholdNumbers() = default;
  LD2402MoveThresholdNumbers(uint8_t gate);

 protected:
  uint8_t gate_;
  void control(float move_threshold) override;
};

}  // namespace esphome::ld2402
