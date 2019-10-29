/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/baseGyroAngleAt.h"

baseGyroAngleAt::baseGyroAngleAt(double position)
{
  setpoint = position;
  Requires(&Robot::m_base);
}

void baseGyroAngleAt::Initialize() {
Robot::m_base.GyroZero();
}

void baseGyroAngleAt::Execute()
{

  Robot::m_base.GyroAngleAt(setpoint);
  if (Robot::m_base.gyroCurr > setpoint - uncertainty && Robot::m_base.gyroCurr < setpoint + uncertainty)
    timeInBounds->Start();

  else
  {
    timeInBounds->Stop();
    timeInBounds->Reset();
  }
}

// Make this return true when this Command no longer needs to run execute()
bool baseGyroAngleAt::IsFinished() { return timeInBounds->Get() > 1; }

// Called once after isFinished returns true
void baseGyroAngleAt::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void baseGyroAngleAt::Interrupted() {}
