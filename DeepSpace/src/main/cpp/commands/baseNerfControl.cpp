/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/baseNerfControl.h"

baseNerfControl::baseNerfControl() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(&Robot::m_base);
}

// Called just before this Command runs the first time
void baseNerfControl::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void baseNerfControl::Execute() {
  Robot::m_base.NerfJoystickControl();
}

// Make this return true when this Command no longer needs to run execute()
bool baseNerfControl::IsFinished() { return false; }

// Called once after isFinished returns true
void baseNerfControl::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void baseNerfControl::Interrupted() {}
