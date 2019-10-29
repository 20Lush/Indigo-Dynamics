/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/intakeProbe.h"

intakeProbe::intakeProbe() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(&Robot::m_intake);
}

// Called once when the command executes
void intakeProbe::Initialize() {

    if(!Robot::m_intake.isProbed){
    Robot::m_intake.DisengageProbe();
    Robot::m_intake.isProbed = true;
    }

    else if(Robot::m_intake.isProbed){
    Robot::m_intake.EngageProbe();
    Robot::m_intake.isProbed = false;
    }

}