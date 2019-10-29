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

class Lift : public frc::Subsystem{
private:

frc::DoubleSolenoid *front = new frc::DoubleSolenoid(kMasterPCMID, kFrontLiftForward, kFrontLiftReverse);
frc::DoubleSolenoid *rear = new frc::DoubleSolenoid(kMasterPCMID, kRearLiftForward, kRearLiftReverse);

public:
  Lift();
  void InitDefaultCommand() override;

  void pneuSet(int send);
  void motSet(int send);
};
