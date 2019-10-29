/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/ArmMotorStage.h"

#include "commands/armControl.h"

ArmMotorStage::ArmMotorStage() : Subsystem("ArmMotorStage") {

armRight->SelectProfileSlot(0,0);
armRight->Config_kP(0, 5.5);
armRight->Config_kI(0, 0);
armRight->Config_kD(0, 0.4);
armRight->Config_kF(0, 0.3);
armRight->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder);
armRight->SetSensorPhase(false);
armRight->SetInverted(true);

sendPosition = armRight->GetSelectedSensorPosition();

armLeft->SetInverted(false);
armLeft->Set(ControlMode::Follower, kArmRight);

armRight->Set(ControlMode::Position, 0);

}

void ArmMotorStage::InitDefaultCommand()
{
  SetDefaultCommand(new armControl());
}

void ArmMotorStage::SetTriggerControl(double rightTrigger, double leftTrigger)
{
  armLeft->Set(ControlMode::Follower, kArmRight);
  armRight->Set(ControlMode::PercentOutput, rightTrigger - leftTrigger);
  frc::SmartDashboard::PutNumber("armPos", armRight->GetSelectedSensorPosition());
  
}

void ArmMotorStage::LevelZero(){
  armRight->Set(ControlMode::Position, -75);
  
}

void ArmMotorStage::LevelTwo(){
  sendPosition = armRight->GetSelectedSensorPosition();
  armRight->Set(ControlMode::Position, 850);
}

bool ArmMotorStage::BottomLimitStatus(){

return armRight->GetSensorCollection().IsRevLimitSwitchClosed();

}