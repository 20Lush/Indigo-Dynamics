/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#define _USE_MATH_DEFINES

#include <frc/commands/Subsystem.h>
#include <frc/Spark.h>
#include <RobotMap.h>
#include <frc/AnalogInput.h>
#include <math.h>

class baseDrive : public frc::Subsystem
{
private:
  //HARDWARE DECLERATIONS
  frc::Spark *driveFrontRight = new frc::Spark(kDriveFrontRight);
  frc::Spark *driveFrontLeft = new frc::Spark(kDriveFrontLeft);
  frc::Spark *driveBackRight = new frc::Spark(kDriveBackRight); // sparks >_>
  frc::Spark *driveBackLeft = new frc::Spark(kDriveBackLeft);

  frc::Spark *pivotFrontRight = new frc::Spark(kPivotFrontRight);
  frc::Spark *pivotFrontLeft = new frc::Spark(kPivotFrontLeft);
  frc::Spark *pivotBackRight = new frc::Spark(kPivotBackRight);
  frc::Spark *pivotBackLeft = new frc::Spark(kPivotBackLeft);

  frc::AnalogInput *eFrontRight = new frc::AnalogInput(kEncoderFrontLeft);
  frc::AnalogInput *eFrontLeft = new frc::AnalogInput(kEncoderFrontRight);
  frc::AnalogInput *eBackRight = new frc::AnalogInput(kEncoderBackRight);
  frc::AnalogInput *eBackLeft = new frc::AnalogInput(kEncoderBackLeft);

  //GLOBAL SWERVE PID CONSTANT - EACH WHEEL MIGHT NEED ITS OWN SET BUT I DOUBT IT -ZJ
  double kPSwerve = 0.1; //0.1 is an expiremental value just to see if you can get any motion out of the pivots. Adjust Accordingly.
  double kISwerve = 0.0;
  double kDSwerve = 0.0;

  //FRONT RIGHT VARIABLE SET
  double swerveCurrFR;
  double swervePrevErrFR;
  double swerveErrFR;
  double swerveDerivativeFR;
  double swerveCorrectionFR;
  double swerveSetpointFR;

  //FRONT LEFT VARIABLE SET
  double swerveCurrFL;
  double swervePrevErrFL;
  double swerveErrFL;
  double swerveDerivativeFL;
  double swerveCorrectionFL;
  double swerveSetpointFL;

  //BACK RIGHT VARIABLE SET
  double swerveCurrBR;
  double swervePrevErrBR;
  double swerveErrBR;
  double swerveDerivativeBR;
  double swerveCorrectionBR;
  double swerveSetpointBR;

  //BACK LEFT VARIABLE SET
  double swerveCurrBL;
  double swervePrevErrBL;
  double swerveErrBL;
  double swerveDerivativeBL;
  double swerveCorrectionBL;
  double swerveSetpointBL;

public:
  double encoderFRDiagnostic();
  double encoderFLDiagnostic();
  double encoderBRDiagnostic();
  double encoderBLDiagnostic();

  double angularVector();
  double translationVector();

  baseDrive();
  void InitDefaultCommand() override;
  void xyJoystickControl(double inputX1, double inputY, double inputX2);
  void encoderZero();
};
