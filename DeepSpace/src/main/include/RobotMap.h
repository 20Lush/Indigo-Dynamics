/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */

// For example to map the left and right motors, you could define the
// following variables to use with your drivetrain subsystem.
// constexpr int kLeftMotor = 1;
// constexpr int kRightMotor = 2;
constexpr int kFrontLeft = 7;
constexpr int kFrontRight = 4;
constexpr int kBackLeft = 10;
constexpr int kBackRight = 8;

constexpr int kArmLeft = 12;
constexpr int kArmRight = 11;

constexpr int kMasterPCMID = 0;
constexpr int kSlavePCMID = 1;

constexpr int kArmPneuLeftForward = 0;
constexpr int kArmPneuLeftReverse = 1;
constexpr int kArmPneuRightForward = 2;
constexpr int kArmPneuRightReverse = 3;

constexpr int kIntakeWristForward = 6;
constexpr int kIntakeWristReverse = 7;
constexpr int kIntakeWristMotor = 5;

constexpr int kIntakeRollerRight = 6;
constexpr int kIntakeRollerLeft = 9; //nice
constexpr int kIntakePunchLeftForward = 4;
constexpr int kIntakePunchLeftReverse = 5;

constexpr int kWristLimit = 5000;
constexpr int kEndBuffer = 750;

constexpr int kFrontLiftForward = 2; //CHANGE ON DAVIS BOT
constexpr int kFrontLiftReverse = 3; //CHANGE ON DAVIS BOT
constexpr int kRearLiftForward = 0; //CHANGE ON DAVIS BOT
constexpr int kRearLiftReverse = 1; //CHANGE ON DAVIS BOT

// If you are using multiple modules, make sure to define both the port
// number and the module. For example you with a rangefinder:
// constexpr int kRangeFinderPort = 1;
// constexpr int kRangeFinderModule = 1;
