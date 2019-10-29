/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/CommandGroup.h>
#include <commands/sequenceBackOff.h>
#include <commands/sequenceLoad.h>
#include <commands/intakeProbeEngage.h>
#include <commands/baseApproachLoad.h>
#include <commands/armGoToTwo.h>

class sequenceLevelThreeScore : public frc::CommandGroup {
 public:
  sequenceLevelThreeScore();
};
