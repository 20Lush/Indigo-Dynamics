/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <frc/commands/Scheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

BaseDrive Robot::m_base;
ArmPneuStage Robot::m_pneustage;
ArmMotorStage Robot::m_motorstage;
OI Robot::m_oi;
IntakeWrist Robot::m_intakewrist;
Intake Robot::m_intake;
Lift Robot::m_lift;


void Robot::RobotInit()
{
  // m_chooser.SetDefaultOption("Default Auto", &m_defaultAuto);
  // m_chooser.AddOption("My Auto", &m_myAuto);
  frc::SmartDashboard::PutData("ToggleProbe", new intakeProbe());
  frc::SmartDashboard::PutData("Up", new wristGoToLevel(0));
  frc::SmartDashboard::PutData("Mid", new wristGoToLevel(1));
  frc::SmartDashboard::PutData("Down", new wristGoToLevel(2));
  frc::SmartDashboard::PutData("3", new armGoToTwo());
  frc::SmartDashboard::PutData("reset wrist", new wristReset());
}

void Robot::RobotPeriodic()
{
  // TODO debuging stuff
  frc::SmartDashboard::PutNumber("wristPosition", Robot::m_intakewrist.sendPosition());
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() { frc::Scheduler::GetInstance()->Run(); }

void Robot::AutonomousInit()
{
  // std::string autoSelected = frc::SmartDashboard::GetString(
  //     "Auto Selector", "Default");
  // if (autoSelected == "My Auto") {
  //   m_autonomousCommand = &m_myAuto;
  // } else {
  //   m_autonomousCommand = &m_defaultAuto;
  // }

  Robot::m_intakewrist.encoderZero();
}

void Robot::AutonomousPeriodic() { frc::Scheduler::GetInstance()->Run(); }

void Robot::TeleopInit()
{
  nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetEntry("camMode").SetDouble(0);
  nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetEntry("ledMode").SetDouble(3);
}

void Robot::TeleopPeriodic() { frc::Scheduler::GetInstance()->Run(); }

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
