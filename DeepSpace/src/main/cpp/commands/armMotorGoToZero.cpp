/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/armMotorGoToZero.h"
#include <iostream>

armMotorGoToZero::armMotorGoToZero()
{
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(&Robot::m_motorstage);
}

// Called just before this Command runs the first time
void armMotorGoToZero::Initialize()
{
  Robot::m_motorstage.LevelZero();
  std::cout << "Position control started";
}

void armMotorGoToZero::Execute()
{
  position = Robot::m_motorstage.sendPosition;
}

// Make this return true when this Command no longer needs to run execute()
bool armMotorGoToZero::IsFinished()
{
return Robot::m_motorstage.BottomLimitStatus();
}

void armMotorGoToZero::End() {}
