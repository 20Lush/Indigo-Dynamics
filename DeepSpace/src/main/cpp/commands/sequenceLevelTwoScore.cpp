/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/sequenceLevelTwoScore.h"

sequenceLevelTwoScore::sequenceLevelTwoScore() {

AddSequential(new baseApproachLoad(2));
AddSequential(new baseGoToDistance(0,0));
AddSequential(new armPneuControlUp());
AddSequential(new frc::WaitCommand(2));
AddSequential(new baseGoToDistance(5000, 0));
AddSequential(new intakeProbeDisengage());
AddSequential(new sequenceBackOffMedium());

}
