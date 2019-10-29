/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/intakeRollersStop.h"

intakeRollersStop::intakeRollersStop() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(&Robot::m_intake);
  Requires(&Robot::m_intakewrist);
  Requires(&Robot::m_base);
}

// Called once when the command executes
void intakeRollersStop::Initialize() {

Robot::m_intake.RollersStop();
Robot::m_intakewrist.kill();
Robot::m_base.SetJoystickControl();

}
