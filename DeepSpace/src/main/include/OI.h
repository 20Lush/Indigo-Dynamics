/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <frc/Joystick.h>
#include <frc/buttons/JoystickButton.h>

class OI
{
public:
  OI();
  frc::Joystick *js1 = new frc::Joystick(0);
  frc::Joystick *js2 = new frc::Joystick(1);
  frc::Joystick *js3 = new frc::Joystick(2);
  frc::JoystickButton *a = new frc::JoystickButton(js1, 1),
                      *b = new frc::JoystickButton(js1, 2),
                      *x = new frc::JoystickButton(js1, 3),
                      *y = new frc::JoystickButton(js1, 4),
                      *rb = new frc::JoystickButton(js1, 6),
                      *lb = new frc::JoystickButton(js1, 5);

  frc::JoystickButton *REplus = new frc::JoystickButton(js2, 3),
                      *FRplus = new frc::JoystickButton(js2, 4),
                      *stp = new frc::JoystickButton(js2, 5),
                      *out = new frc::JoystickButton(js2, 6),
                      *load = new frc::JoystickButton(js2, 7),
                      *lv1 = new frc::JoystickButton(js2, 8),
                      *down = new frc::JoystickButton(js2, 10),
                      *in = new frc::JoystickButton(js2, 12),
                      *mid = new frc::JoystickButton(js2, 11),
                      *probe = new frc::JoystickButton(js2, 2);

  frc::JoystickButton *pneuUp = new frc::JoystickButton(js3, 1),
                      *tog = new frc::JoystickButton(js3, 2),
                      *pneuDown = new frc::JoystickButton(js3, 3),
                      *armDown = new frc::JoystickButton(js3, 4),
                      *armUp = new frc::JoystickButton(js3, 5),
                      *reset = new frc::JoystickButton(js3, 9),
                      *REminus = new frc::JoystickButton(js3, 10),
                      *FRminus = new frc::JoystickButton(js3, 11);
};