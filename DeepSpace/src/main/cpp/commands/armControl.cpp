/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/armControl.h"
#include <iostream>

armControl::armControl() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(&Robot::m_motorstage);
}

// Called just before this Command runs the first time
void armControl::Initialize() {
  std::cout << "Manual Controll started";
}

// Called repeatedly when this Command is scheduled to run
void armControl::Execute() {
  double rightTrigger = 0; //Robot::m_oi.js1->GetRawAxis(3);
  double leftTrigger = 0; //Robot::m_oi.js1->GetRawAxis(2);

  Robot::m_motorstage.SetTriggerControl(rightTrigger, leftTrigger);

}

// Make this return true when this Command no longer needs to run execute()
bool armControl::IsFinished() { return false; }

// Called once after isFinished returns true
void armControl::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void armControl::Interrupted() {}
