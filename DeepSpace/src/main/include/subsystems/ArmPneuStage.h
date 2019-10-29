/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Subsystem.h>
#include <frc/DoubleSolenoid.h>
#include <RobotMap.h>

class ArmPneuStage : public frc::Subsystem {
 private:
  // It's desirable that everything possible under private except
  // for methods that implement subsystem capabilities

frc::DoubleSolenoid *rightPneu = new frc::DoubleSolenoid(kSlavePCMID, kArmPneuRightForward, kArmPneuRightReverse);
frc::DoubleSolenoid *leftPneu = new frc::DoubleSolenoid(kSlavePCMID, kArmPneuLeftForward, kArmPneuLeftReverse);


 public:
  ArmPneuStage();
  void InitDefaultCommand() override;
  void PneuStageUp();
  void PneuStageDown();
  void PneuStageVent();
};
