/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/ArmPneuStage.h"

ArmPneuStage::ArmPneuStage() : Subsystem("ExampleSubsystem") {}

void ArmPneuStage::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
}

void ArmPneuStage::PneuStageUp(){

rightPneu->Set(frc::DoubleSolenoid::kReverse);
leftPneu->Set(frc::DoubleSolenoid::kForward);

}

void ArmPneuStage::PneuStageDown(){

  rightPneu->Set(frc::DoubleSolenoid::kForward);
  leftPneu->Set(frc::DoubleSolenoid::kReverse);

}

void ArmPneuStage::PneuStageVent(){
  rightPneu->Set(frc::DoubleSolenoid::kOff);
  leftPneu->Set(frc::DoubleSolenoid::kOff);
}

