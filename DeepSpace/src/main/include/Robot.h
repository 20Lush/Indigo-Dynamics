/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "OI.h"
#include <frc/TimedRobot.h>
#include <frc/commands/Command.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "commands/baseControl.h"
#include "commands/baseGoToDistance.h"
#include "commands/intakeRollersIn.h"
#include "commands/intakeRollersOut.h"
#include "commands/intakeProbe.h"
#include "commands/sequenceBackOff.h"
#include "commands/wristGoToLevel.h"
#include "commands/baseApproachLoad.h"
#include "commands/sequenceLoad.h"
#include "commands/sequenceLeveOneScore.h"
#include "commands/baseGyroAngleAt.h"
#include "commands/baseLockRotation.h"
#include "commands/sequenceLevelTwoScore.h"
#include "commands/sequenceLevelThreeScore.h"
#include "commands/armGoToTwo.h"
#include "commands/armControl.h"
#include "commands/wristReset.h"

#include "subsystems/BaseDrive.h"
#include "subsystems/ArmMotorStage.h"
#include "subsystems/ArmPneuStage.h"
#include "subsystems/IntakeWrist.h"
#include "subsystems/Intake.h"
#include "subsystems/Lift.h"

#include <iostream>

class Robot : public frc::TimedRobot {
 public:
  static BaseDrive m_base;
  static ArmMotorStage m_motorstage;
  static OI m_oi;
  static ArmPneuStage m_pneustage;
  static IntakeWrist m_intakewrist;
  static Intake m_intake;
  static Lift m_lift;

  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

 private:
  // Have it null by default so that if testing teleop it
  // doesn't have undefined behavior and potentially crash.
  frc::Command* m_autonomousCommand = nullptr;
  // ExampleCommand m_defaultAuto;
  // MyAutoCommand m_myAto;
  frc::SendableChooser<frc::Command*> m_chooser;
};
