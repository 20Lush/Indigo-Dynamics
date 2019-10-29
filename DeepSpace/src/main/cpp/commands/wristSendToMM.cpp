/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/wristSendToMM.h"

wristSendToMM::wristSendToMM(double position)
{
  Requires(&Robot::m_intakewrist);
  m_position = position;
  upperLimit = m_position + kWristLimit;
  lowerLimit = m_position - kWristLimit;
}

// Called just before this Command runs the first time
void wristSendToMM::Initialize()
{
  Robot::m_intakewrist.Ping();
}

// Called repeatedly when this Command is scheduled to run
void wristSendToMM::Execute()
{
  if (Robot::m_intakewrist.sendPosition() < upperLimit && Robot::m_intakewrist.sendPosition() > lowerLimit)
  {
    Robot::m_intakewrist.Return();
    Robot::m_intakewrist.ControlUp(-0.3);
  }

  else
  {
    Robot::m_intakewrist.wristPositionArbitrary(m_position);
    Robot::m_intakewrist.Ping();
  }
}

// Make this return true when this Command no longer needs to run execute()
bool wristSendToMM::IsFinished() { return Robot::m_intakewrist.sendPosition() < upperLimit - kEndBuffer && Robot::m_intakewrist.sendPosition() > lowerLimit + kEndBuffer; }

// Called once after isFinished returns true
void wristSendToMM::End() { Robot::m_intakewrist.ControlUp(0.0);}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void wristSendToMM::Interrupted() {Robot::m_intakewrist.ControlUp(0.0);}
