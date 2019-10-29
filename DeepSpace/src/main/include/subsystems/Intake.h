/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Subsystem.h>
#include <frc/DoubleSolenoid.h>
#include <ctre/Phoenix.h>
#include <RobotMap.h>

class Intake : public frc::Subsystem {
 private:
  // It's desirable that everything possible under private except
  // for methods that implement subsystem capabilities

frc::DoubleSolenoid *punchLeft = new frc::DoubleSolenoid(kMasterPCMID, kIntakePunchLeftForward, kIntakePunchLeftReverse);
TalonSRX *leftRoller = new TalonSRX(kIntakeRollerLeft);
TalonSRX *rightRoller = new TalonSRX(kIntakeRollerRight);

 public:
  Intake();
  bool isPinged = false;
  bool isProbed = true;
  void InitDefaultCommand() override;
  void RollersIn();
  void RollersOut();
  void RollersStop();
  void EngageProbe();
  void DisengageProbe();

};
