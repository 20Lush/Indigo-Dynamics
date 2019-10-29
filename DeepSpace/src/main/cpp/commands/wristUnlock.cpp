/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/wristUnlock.h"

wristUnlock::wristUnlock() {
  Requires(&Robot::m_intakewrist);

}

// Called just before this Command runs the first time
void wristUnlock::Initialize() {
  timer->Stop();
  timer->Reset();
  timer->Start();
  Robot::m_intakewrist.Ping();
}

// Called repeatedly when this Command is scheduled to run
void wristUnlock::Execute() {Robot::m_intakewrist.ControlUp(0.45);}

// Make this return true when this Command no longer needs to run execute()
bool wristUnlock::IsFinished() { return timer->Get() > 0.2; }

// Called once after isFinished returns true
void wristUnlock::End() {
  Robot::m_intakewrist.ControlUp(0.0);
  timer->Stop();
  timer->Reset();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void wristUnlock::Interrupted() {
  Robot::m_intakewrist.ControlUp(0.0);
  timer->Stop();
  timer->Reset();
}
