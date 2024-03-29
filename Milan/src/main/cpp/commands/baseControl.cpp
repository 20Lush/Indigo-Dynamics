/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/baseControl.h"

baseControl::baseControl() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(&Robot::m_basedrive);
}

// Called just before this Command runs the first time
void baseControl::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void baseControl::Execute() {
leftStickX = Robot::m_oi.js1->GetRawAxis(0);
leftStickY = Robot::m_oi.js1->GetRawAxis(1);
rightStickX = Robot::m_oi.js1->GetRawAxis(4);

Robot::m_basedrive.xyJoystickControl(leftStickX, leftStickY, rightStickX);

}

// Make this return true when this Command no longer needs to run execute()
bool baseControl::IsFinished() { return false; }

// Called once after isFinished returns true
void baseControl::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void baseControl::Interrupted() {}
