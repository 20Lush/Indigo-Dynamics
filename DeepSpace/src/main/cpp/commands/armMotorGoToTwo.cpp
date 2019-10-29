/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/armMotorGoToTwo.h"

armMotorGoToTwo::armMotorGoToTwo()
{
  Requires(&Robot::m_motorstage);
}

// Called just before this Command runs the first time
void armMotorGoToTwo::Initialize(){}

void armMotorGoToTwo::Execute()
{
    Robot::m_motorstage.LevelTwo();
}

// Make this return true when this Command no longer needs to run execute()
bool armMotorGoToTwo::IsFinished()
{
return false;//Robot::m_motorstage.sendPosition > 790;
}

void armMotorGoToTwo::End() {}