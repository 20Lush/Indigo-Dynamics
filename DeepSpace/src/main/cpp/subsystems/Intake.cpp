/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Intake.h"
#include "commands/intakeProbe.h"

Intake::Intake() : Subsystem("ExampleSubsystem") {



}

void Intake::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
}

void Intake::RollersIn(){
  leftRoller->Set(ControlMode::PercentOutput, 0.3);
  rightRoller->Set(ControlMode::PercentOutput, -0.3);
}

void Intake::RollersOut(){
  leftRoller->Set(ControlMode::PercentOutput, -1);
  rightRoller->Set(ControlMode::PercentOutput, 1);
}

void Intake::RollersStop(){

  leftRoller->Set(ControlMode::PercentOutput, 0);
  rightRoller->Set(ControlMode::PercentOutput, 0);

}

void Intake::EngageProbe(){

punchLeft->Set(frc::DoubleSolenoid::kReverse);

}

void Intake::DisengageProbe(){

punchLeft->Set(frc::DoubleSolenoid::kForward);

}
