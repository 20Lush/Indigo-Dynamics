/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/armPneuControlUp.h"

armPneuControlUp::armPneuControlUp() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(&Robot::m_pneustage);
}

// Called just before this Command runs the first time
void armPneuControlUp::Initialize() {

Robot::m_pneustage.PneuStageUp();
  status = false;
}

// Make this return true when this Command no longer needs to run execute()
bool armPneuControlUp::IsFinished() { 
  if(!status)
  return true;
 }

void armPneuControlUp::End() {}

void armPneuControlUp::Interrupted() {}


