/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/wristGoToLevel.h"

wristGoToLevel::wristGoToLevel(int sentLevel) {
  int level = 0;
  if(sentLevel == 0)
  level = 0;
  else if (sentLevel == 1)
  level = -34000;
  else if (sentLevel == 2)
  level = -90000;
  else
  level = 0;

AddSequential(new wristUnlock());
AddSequential(new wristSendToMM(level));

}
