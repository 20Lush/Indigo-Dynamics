/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/CommandGroup.h>
#include <commands/baseGoToDistance.h>
#include <commands/armGoToZero.h>
#include <commands/intakeProbeEngage.h>
#include <commands/intakeProbeDisengage.h>
#include <commands/armPneuControlDown.h>

class sequenceBackOffMedium : public frc::CommandGroup {
 public:
  sequenceBackOffMedium();
};
