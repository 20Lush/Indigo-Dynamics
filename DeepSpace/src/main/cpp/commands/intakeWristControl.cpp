/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/intakeWristControl.h"

intakeWristControl::intakeWristControl()
{
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(&Robot::m_intakewrist);
}

// Called just before this Command runs the first time
void intakeWristControl::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void intakeWristControl::Execute()
{
    Robot::m_intakewrist.ControlUp(0.0);

  // TODO not sure what this is doing? Is there a need for this?
     if(!Robot::m_intake.isPinged)
     Robot::m_intakewrist.Return();

     else if(Robot::m_intake.isPinged)
     Robot::m_intakewrist.Ping();

}

// Make this return true when this Command no longer needs to run execute()
bool intakeWristControl::IsFinished() { return false; }

// Called once after isFinished returns true
void intakeWristControl::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void intakeWristControl::Interrupted() {}
