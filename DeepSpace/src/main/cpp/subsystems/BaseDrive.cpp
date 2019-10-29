/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/BaseDrive.h"
#include "commands/baseControl.h"
#include "frc/smartdashboard/SmartDashboard.h"

BaseDrive::BaseDrive() : Subsystem("BaseDrive")
{
	backLeft->SelectProfileSlot(0, 0);
	backLeft->Config_kP(0, 0.41);
	backLeft->Config_kI(0, 0.0);
	backLeft->Config_kD(0, 0.4);
	backLeft->Config_kF(0, 0.0);
	backLeft->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute);
	backLeft->SetSensorPhase(true); //SET TRUE IN COMPETITION ROBOT

	frontRight->SetInverted(true);
	backRight->SetInverted(true);
}

void BaseDrive::InitDefaultCommand()
{
	// Set the default command for a subsystem here.
	SetDefaultCommand(new baseControl());
	
}

void BaseDrive::SetJoystickControl()
{

	turnPow = Robot::m_oi.js1->GetRawAxis(1) * -1 * 0.90;
	fwdPow = Robot::m_oi.js1->GetRawAxis(4) * 0.80;
	if (turnPow < 0.1 && turnPow > -0.1) // if turnPow isn't large enough
		modTP = 0;
	else
		modTP = turnPow; // * turnPow;
	
	frontLeft->Set(ControlMode::PercentOutput, modTP + fwdPow);
	frontRight->Set(ControlMode::PercentOutput, modTP - fwdPow);
	backLeft->Set(ControlMode::PercentOutput, modTP + fwdPow);
	backRight->Set(ControlMode::PercentOutput, modTP - fwdPow);
	
	frc::SmartDashboard::PutNumber("encoder", backLeft->GetSelectedSensorPosition());
frc::SmartDashboard::PutNumber("gyro", baseGyro->GetAngle());
}

void BaseDrive::SetEncoderControl(double distance, double angle)
{

	EncoderPosition = backLeft->GetSelectedSensorPosition();

	frc::SmartDashboard::PutNumber("encoderpos", backLeft->GetOutputCurrent());
	frc::SmartDashboard::PutNumber("Forward pow", modTP);

	gyroSetPoint = angle; //Gyro algo parts
	gyroCurr = baseGyro->GetAngle();
	gyroErr = gyroCurr - gyroSetPoint;
	gyroDerivative = gyroErr - gyroPrevErr;
	gyroPrevErr = gyroErr;

	setpoint = distance; //Drive algo parts
	driveCurr = backLeft->GetSelectedSensorPosition();
	driveErr = driveCurr - setpoint;
	driveDerivative = backLeft->GetErrorDerivative();

	fwdPow = kPGyro * gyroErr + kDGyro * gyroDerivative;	//send Rotation Corrections
	modTP = kPDrive * driveErr + kIDrive * driveDerivative; //send Distance Corrections

	frontLeft->Set(ControlMode::PercentOutput, -modTP + fwdPow); //Master Motor Calc
	frontRight->Set(ControlMode::PercentOutput, -modTP - fwdPow);
	backLeft->Set(ControlMode::PercentOutput, -modTP + fwdPow);
	backRight->Set(ControlMode::PercentOutput, -modTP - fwdPow);
}

void BaseDrive::NerfJoystickControl()
{
	turnPow = Robot::m_oi.js1->GetRawAxis(1) * -1 * 0.5;
	fwdPow = Robot::m_oi.js1->GetRawAxis(4) * 0.45;
	if (turnPow < 0.1 && turnPow > -0.1) // if turnPow isn't large enough
		modTP = 0;
	else
		modTP = turnPow; // * turnPow;

	frontLeft->Set(ControlMode::PercentOutput, modTP + fwdPow);
	frontRight->Set(ControlMode::PercentOutput, modTP - fwdPow);
	backLeft->Set(ControlMode::PercentOutput, modTP + fwdPow);
	backRight->Set(ControlMode::PercentOutput, modTP - fwdPow);

}

void BaseDrive::ZeroEncoder()
{
	backLeft->SetSelectedSensorPosition(0);
}

void BaseDrive::SetLocker()
{

	turnPow = Robot::m_oi.js1->GetRawAxis(1) * -1;
	tX = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0);
	frc::SmartDashboard::PutNumber("tX", tX);

	fwdPow = tX * kPX;

	frontLeft->Set(ControlMode::PercentOutput, modTP + fwdPow);
	frontRight->Set(ControlMode::PercentOutput, modTP - fwdPow);
	backLeft->Set(ControlMode::PercentOutput, modTP + fwdPow);
	backRight->Set(ControlMode::PercentOutput, modTP - fwdPow);
}

void BaseDrive::LoadingApproach()
{
	tX = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0);
	tY = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ty", 0);
	frc::SmartDashboard::PutNumber("tX", tX);
	frc::SmartDashboard::PutNumber("tY", tY);

	tY_Derivative = tY - PrevtY;
	PrevtY = tY;

	fwdPow = tX * kPX;
	modTP = tY * kPY + tY_Derivative * kDY;

	frontLeft->Set(ControlMode::PercentOutput, modTP + fwdPow);
	frontRight->Set(ControlMode::PercentOutput, modTP - fwdPow);
	backLeft->Set(ControlMode::PercentOutput, modTP + fwdPow);
	backRight->Set(ControlMode::PercentOutput, modTP - fwdPow);
}

void BaseDrive::GyroAngleAt(double position)
{
	gyroSetPoint = position;
	gyroCurr = baseGyro->GetAngle();
	gyroErr = gyroCurr - gyroSetPoint;
	gyroDerivative = gyroErr - gyroPrevErr;
	gyroPrevErr = gyroErr;

	fwdPow = kPGyro * gyroErr + kDGyro * gyroDerivative;

	frontLeft->Set(ControlMode::PercentOutput, modTP + fwdPow);
	frontRight->Set(ControlMode::PercentOutput, modTP - fwdPow);
	backLeft->Set(ControlMode::PercentOutput, modTP + fwdPow);
	backRight->Set(ControlMode::PercentOutput, modTP - fwdPow);
}

void BaseDrive::GyroZero()
{
	baseGyro->Reset();
}

void BaseDrive::CamDriveMode()
{
	nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetEntry("camMode").SetDouble(1);
	nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetEntry("ledMode").SetDouble(1);
}

void BaseDrive::CamTrackMode()
{
	nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetEntry("camMode").SetDouble(0);
	nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetEntry("ledMode").SetDouble(3);
}