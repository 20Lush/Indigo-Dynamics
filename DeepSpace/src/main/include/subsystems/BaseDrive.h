/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Subsystem.h>
#include <ctre/Phoenix.h>
#include <RobotMap.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/AnalogGyro.h>

class BaseDrive : public frc::Subsystem
{
private:
  // It's desirable that everything possible under private except
  // for methods that implement subsystem capabilities
  TalonSRX *frontLeft = new TalonSRX(kFrontLeft);
  TalonSRX *backLeft = new TalonSRX(kBackLeft);
  TalonSRX *frontRight = new TalonSRX(kFrontRight);
  TalonSRX *backRight = new TalonSRX(kBackRight);

  frc::AnalogGyro *baseGyro = new frc::AnalogGyro(0);

  double fwdPow;
  double modTP;
  double turnPow;

  double kPDrive = 0.00009;
  double kIDrive = 0.0;
  double kDDrive = 0.0;
  double prevDriveErr;
  double driveErr;
  double driveDerivative;
  double setpoint;

  double kPX = 0.05;
  double kIX = 0.0;
  double kDX = 0.2;
  double kPY = 0.091;
  double kIY = 0.0;
  double kDY = 0.0;
  double PrevtY;
  double tY_Derivative;

  double gyroSetPoint;
  double kPGyro = 0.075;
  double kIGyro = 0.0;
  double kDGyro = 0.06;
  double gyroErr;
  double gyroPrevErr;
  double gyroDerivative;

public:
  double gyroCurr;
  double driveCurr;
  double tX;
  double tY;
  BaseDrive();
  double EncoderPosition;
  void InitDefaultCommand() override;
  void SetJoystickControl();
  void NerfJoystickControl();
  void SetEncoderControl(double distance, double angle);
  void ZeroEncoder();
  void SetLocker();
  void LoadingApproach();
  void GyroAngleAt(double position);
  void GyroZero();
  void CamDriveMode();
  void CamTrackMode();
};
