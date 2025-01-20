/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008-2017. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "frc/wpilib/ahal/InterruptableSensorBase.h"

#include "absl/log/check.h"
#include "absl/log/log.h"

#include "frc/wpilib/ahal/WPIErrors.h"
#include "hal/HAL.h"

using namespace frc;

InterruptableSensorBase::InterruptableSensorBase() {}

void InterruptableSensorBase::RequestInterrupts() {
  if (StatusIsFatal()) return;

  CHECK_EQ(m_interrupt, HAL_kInvalidHandle);
  AllocateInterrupts();
  if (StatusIsFatal()) return;  // if allocate failed, out of interrupts

  int32_t status = 0;
  HAL_RequestInterrupts(
      m_interrupt, GetPortHandleForRouting(),
      static_cast<HAL_AnalogTriggerType>(GetAnalogTriggerTypeForRouting()),
      &status);
  wpi_setErrorWithContext(status, HAL_GetErrorMessage(status));
  SetUpSourceEdge(true, false);
}

void InterruptableSensorBase::AllocateInterrupts() {
  CHECK_EQ(m_interrupt, HAL_kInvalidHandle);
  // Expects the calling leaf class to allocate an interrupt index.
  int32_t status = 0;
  m_interrupt = HAL_InitializeInterrupts(&status);
  wpi_setErrorWithContext(status, HAL_GetErrorMessage(status));
}

void InterruptableSensorBase::CancelInterrupts() {
  if (StatusIsFatal()) return;
  CHECK_NE(m_interrupt, HAL_kInvalidHandle);
  HAL_CleanInterrupts(m_interrupt);
  // ignore status, as an invalid handle just needs to be ignored.
  m_interrupt = HAL_kInvalidHandle;
}

InterruptableSensorBase::WaitResult InterruptableSensorBase::WaitForInterrupt(
    double timeout, bool ignorePrevious) {
  if (StatusIsFatal()) return InterruptableSensorBase::kTimeout;
  CHECK_NE(m_interrupt, HAL_kInvalidHandle);
  int32_t status = 0;
  int result;

  result = HAL_WaitForInterrupt(m_interrupt, timeout, ignorePrevious, &status);
  wpi_setErrorWithContext(status, HAL_GetErrorMessage(status));

  // Rising edge result is the interrupt bit set in the byte 0xFF
  // Falling edge result is the interrupt bit set in the byte 0xFF00
  // Set any bit set to be true for that edge, and AND the 2 results
  // together to match the existing enum for all interrupts
  int32_t rising = (result & 0xFF) ? 0x1 : 0x0;
  int32_t falling = ((result & 0xFF00) ? 0x0100 : 0x0);
  return static_cast<WaitResult>(falling | rising);
}

hal::fpga_clock::time_point InterruptableSensorBase::ReadRisingTimestamp() {
  if (StatusIsFatal()) return hal::fpga_clock::min_time;
  CHECK_NE(m_interrupt, HAL_kInvalidHandle);
  int32_t status = 0;
  uint64_t timestamp = HAL_ReadInterruptRisingTimestamp(m_interrupt, &status);
  timestamp = HAL_ExpandFPGATime(timestamp, &status);
  wpi_setErrorWithContext(status, HAL_GetErrorMessage(status));
  return hal::fpga_clock::time_point(hal::fpga_clock::duration(timestamp));
}

hal::fpga_clock::time_point InterruptableSensorBase::ReadFallingTimestamp() {
  if (StatusIsFatal()) return hal::fpga_clock::min_time;
  CHECK_NE(m_interrupt, HAL_kInvalidHandle);
  int32_t status = 0;
  uint64_t timestamp = HAL_ReadInterruptFallingTimestamp(m_interrupt, &status);
  timestamp = HAL_ExpandFPGATime(timestamp, &status);
  wpi_setErrorWithContext(status, HAL_GetErrorMessage(status));
  return hal::fpga_clock::time_point(hal::fpga_clock::duration(timestamp));
}

void InterruptableSensorBase::SetUpSourceEdge(bool risingEdge,
                                              bool fallingEdge) {
  if (StatusIsFatal()) return;
  if (m_interrupt == HAL_kInvalidHandle) {
    wpi_setWPIErrorWithContext(
        NullParameter,
        "You must call RequestInterrupts before SetUpSourceEdge");
    return;
  }
  if (m_interrupt != HAL_kInvalidHandle) {
    int32_t status = 0;
    HAL_SetInterruptUpSourceEdge(m_interrupt, risingEdge, fallingEdge, &status);
    wpi_setErrorWithContext(status, HAL_GetErrorMessage(status));
  }
}
