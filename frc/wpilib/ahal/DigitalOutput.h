/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008-2017. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <memory>
#include <string>

#include "frc/wpilib/ahal/DigitalSource.h"
#include "hal/Types.h"

namespace frc {

/**
 * Class to write to digital outputs.
 * Write values to the digital output channels. Other devices implemented
 * elsewhere will allocate channels automatically so for those devices it
 * shouldn't be done here.
 */
class DigitalOutput : public DigitalSource {
 public:
  explicit DigitalOutput(int channel);
  virtual ~DigitalOutput();
  void Set(bool value);
  bool Get() const;
  int GetChannel() const override;
  void Pulse(double length);
  bool IsPulsing() const;
  void SetPWMRate(double rate);
  void EnablePWM(double initialDutyCycle);
  void DisablePWM();
  void UpdateDutyCycle(double dutyCycle);

  // Digital Source Interface
  HAL_Handle GetPortHandleForRouting() const override;
  AnalogTriggerType GetAnalogTriggerTypeForRouting() const override;
  bool IsAnalogTrigger() const override;

 private:
  int m_channel;
  HAL_DigitalHandle m_handle;
  HAL_DigitalPWMHandle m_pwmGenerator;
};

}  // namespace frc
