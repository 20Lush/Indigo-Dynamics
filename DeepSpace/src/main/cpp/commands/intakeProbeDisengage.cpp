/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/intakeProbeDisengage.h"

intakeProbeDisengage::intakeProbeDisengage() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());4
  Requires(&Robot::m_intake);
}

// Called once when the command executes
void intakeProbeDisengage::Initialize() {

    Robot::m_intake.DisengageProbe();

}
