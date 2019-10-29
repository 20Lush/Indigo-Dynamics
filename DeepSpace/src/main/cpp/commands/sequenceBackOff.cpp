/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/sequenceBackOff.h"

sequenceBackOff::sequenceBackOff() {
AddSequential(new intakeProbeDisengage());
AddSequential(new baseGoToDistance(-5000, 0));
AddSequential(new frc::WaitCommand(0.3));
AddSequential(new intakeProbeEngage());
AddSequential(new armGoToZero());

}
