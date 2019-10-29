/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/sequenceLeveOneScore.h"

sequenceLeveOneScore::sequenceLeveOneScore() {
  
AddSequential(new baseApproachLoad(1));
AddSequential(new intakeProbeDisengage());
AddSequential(new sequenceBackOffShort());

}
