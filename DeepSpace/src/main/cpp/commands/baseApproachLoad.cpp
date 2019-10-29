/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/baseApproachLoad.h"

baseApproachLoad::baseApproachLoad(int location) {
  Requires(&Robot::m_base); 
  g_location = location;
}

// Called just before this Command runs the first time
void baseApproachLoad::Initialize() {
  if(g_location == 0)
approachEnd = 1.45;

else if(g_location == 1)
approachEnd = 0.15;

else if(g_location == 2)
approachEnd = 5.8;

}
// Called repeatedly when this Command is scheduled to run
void baseApproachLoad::Execute() {


  Robot::m_base.LoadingApproach();


}

// Make this return true when this Command no longer needs to run execute()
bool baseApproachLoad::IsFinished() { return Robot::m_base.tY < approachEnd; }

// Called once after isFinished returns true
void baseApproachLoad::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void baseApproachLoad::Interrupted() {}
