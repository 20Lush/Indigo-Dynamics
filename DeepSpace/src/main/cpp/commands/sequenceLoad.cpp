 /*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/sequenceLoad.h"

sequenceLoad::sequenceLoad() {
  AddSequential (new limelightModeTrack());
  AddSequential(new intakeProbeDisengage());
  AddSequential(new baseApproachLoad(0));
  AddSequential(new intakeProbeEngage());
  AddSequential(new sequenceBackOffShort());
  
}
