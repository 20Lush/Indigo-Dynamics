/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/liftRearSet.h"

liftRearSet::liftRearSet(int send) {
  // Use Requires() here to declare subsystem dependencies
  Requires(&Robot::m_lift);
  m_send = send;
}

// Called once when the command executes
void liftRearSet::Initialize() {
    Robot::m_lift.motSet(m_send);
}
