/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008-2017. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <cstdio>
#include <iostream>
#include <thread>

#include "aos/realtime.h"
#include "frc/wpilib/ahal/Base.h"
#include "hal/HAL.h"

namespace frc {

class DriverStation;

#define START_ROBOT_CLASS(_ClassName_)                                       \
  int main(int argc, char *argv[]) {                                         \
    aos::InitGoogle(&argc, &argv);                                           \
    /* HAL_Initialize spawns several threads, including the CAN drivers.  */ \
    /* Go to realtime so that the child threads are RT.                   */ \
    aos::SetCurrentThreadRealtimePriority(10);                               \
    if (!HAL_Initialize(500, 0)) {                                           \
      std::cerr << "FATAL ERROR: HAL could not be initialized" << std::endl; \
      return -1;                                                             \
    }                                                                        \
    aos::UnsetCurrentThreadRealtimePriority();                               \
    HAL_Report(HALUsageReporting::kResourceType_Language,                    \
               HALUsageReporting::kLanguage_CPlusPlus);                      \
    static _ClassName_ robot;                                                \
    std::printf("\n********** Robot program starting **********\n");         \
    robot.StartCompetition();                                                \
  }

/**
 * Implement a Robot Program framework.
 * The RobotBase class is intended to be subclassed by a user creating a robot
 * program. Overridden Autonomous() and OperatorControl() methods are called at
 * the appropriate time as the match proceeds. In the current implementation,
 * the Autonomous code will run to completion before the OperatorControl code
 * could start. In the future the Autonomous code might be spawned as a task,
 * then killed at the end of the Autonomous period.
 */
class RobotBase {
 public:
  virtual void StartCompetition() = 0;

 protected:
  RobotBase();
  virtual ~RobotBase() = default;

  RobotBase(const RobotBase &) = delete;
  RobotBase &operator=(const RobotBase &) = delete;

  DriverStation &m_ds;
};

}  // namespace frc
