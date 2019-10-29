/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Lift.h"
#include <commands/liftRearKill.h>

Lift::Lift() : Subsystem("Lift") {}

void Lift::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand()); 
}

void Lift::pneuSet(int send){
  if(send == 0)
  front->Set(frc::DoubleSolenoid::kReverse);


  else
  front->Set(frc::DoubleSolenoid::kForward);
}

void Lift::motSet(int send){
  if(send == 0)
  rear->Set(frc::DoubleSolenoid::kForward);

  else
  rear->Set(frc::DoubleSolenoid::kReverse);
}
