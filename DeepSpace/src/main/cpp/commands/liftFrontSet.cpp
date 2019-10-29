/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/liftFrontSet.h"

liftFrontSet::liftFrontSet(int send) {

  Requires(&Robot::m_lift);
  m_send = send;

}

// Called once when the command executes
void liftFrontSet::Initialize() {
  Robot::m_lift.pneuSet(m_send);
}
