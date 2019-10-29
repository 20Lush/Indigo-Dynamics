/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Command.h>
#include <Robot.h>

class baseGoToDistance : public frc::Command {
 public:

 int m_distance = 0;
 int m_angle = 0;
 double upperLimit;
 double lowerLimit;
  baseGoToDistance(int distance, int angle);
  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
};
