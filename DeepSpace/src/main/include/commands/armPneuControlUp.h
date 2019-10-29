/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Command.h>
#include <Robot.h>

class armPneuControlUp : public frc::Command {
 public:
  armPneuControlUp();
  void Initialize() override;
  bool IsFinished() override;
  void Interrupted() override;
  void End() override;

bool status;
};
