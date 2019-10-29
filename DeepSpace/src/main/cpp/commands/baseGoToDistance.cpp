/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/baseGoToDistance.h"

baseGoToDistance::baseGoToDistance(int distance, int angle)
{
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  m_distance = distance;
  m_angle = angle;
  Requires(&Robot::m_base);
}

void baseGoToDistance::Initialize()
{

  Robot::m_base.ZeroEncoder();
  Robot::m_base.GyroZero();
  upperLimit = m_distance + 1140;
  lowerLimit = m_distance - 1140;
}

void baseGoToDistance::Execute()
{

  Robot::m_base.SetEncoderControl(m_distance, m_angle);
}

bool baseGoToDistance::IsFinished()
{
  return Robot::m_base.EncoderPosition < upperLimit && Robot::m_base.EncoderPosition > lowerLimit;
}