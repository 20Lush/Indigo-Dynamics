/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/armPneuControlDown.h"

armPneuControlDown::armPneuControlDown() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
}

// Called just before this Command runs the first time
void armPneuControlDown::Initialize() {

Robot::m_pneustage.PneuStageDown();
  status = false;

}

bool armPneuControlDown::IsFinished() { 
  if(!status)
  return true;
 }

void armPneuControlDown::End() {}

void armPneuControlDown::Interrupted() {}

