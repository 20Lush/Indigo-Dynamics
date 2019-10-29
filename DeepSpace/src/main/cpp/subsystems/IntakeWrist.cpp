/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/IntakeWrist.h"
#include "commands/intakeWristControl.h"

IntakeWrist::IntakeWrist() : Subsystem("ExampleSubsystem") {

  wristMotor->SelectProfileSlot(0,0); //going Down
  wristMotor->Config_kP(0, 0.004); //MM(0.1)
  wristMotor->Config_kI(0, 0.0);
  wristMotor->Config_kD(0, 0.2);//MM(0.6)
  wristMotor->Config_kF(0, 0);
  wristMotor->EnableCurrentLimit(true);
  wristMotor->ConfigContinuousCurrentLimit(20);
  wristMotor->ConfigPeakCurrentLimit(25);

  wristMotor->SelectProfileSlot(1,0); //going Up
  wristMotor->Config_kP(1, 0.03); //MM(0.1)
  wristMotor->Config_kI(1, 0.0);
  wristMotor->Config_kD(1, 0.01);//MM(0.6)

  wristMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute);
  wristMotor->SetSensorPhase(false); //IF MAG ENCODER FACE DOWN, SET FALSE
  wristMotor->SetInverted(true); //IF MAG ENCODER FACE DOWN, SET TRUE
  wristMotor->ConfigMotionAcceleration((102600/10) * 3);
  wristMotor->ConfigMotionCruiseVelocity((102600/450)*100);


}

void IntakeWrist::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
  SetDefaultCommand(new intakeWristControl());


}

// Put methods for controlling this subsystem
// here. Call these from Commands.
void IntakeWrist::ControlUp(double power){
  frc::SmartDashboard::PutNumber("Wrist power", power);
  wristMotor->Set(ControlMode::PercentOutput, power);
}

void IntakeWrist::Ping(){
  wristLock->Set(frc::DoubleSolenoid::kReverse);
}

void IntakeWrist::Return(){
  wristLock->Set(frc::DoubleSolenoid::kForward);
}

void IntakeWrist::wristPositionDown(){
wristMotor->Set(ControlMode::Position, -102600); 
}

void IntakeWrist::wristPositionMid(){
wristMotor->Set(ControlMode::Position, -29700);
}

void IntakeWrist::wristPositionUp(){
wristMotor->Set(ControlMode::Position, 0);
}

void IntakeWrist::wristPositionArbitrary(double position){
  if(position < wristMotor->GetSelectedSensorPosition()){
    wristMotor->SelectProfileSlot(0,0); //Going Up
  }
  else{
    wristMotor->SelectProfileSlot(1,0); //Going down
  }
  wristMotor->Set(ControlMode::Position, position);
}

double IntakeWrist::sendPosition(){
return wristMotor->GetSelectedSensorPosition();
}

void IntakeWrist::encoderZero(){

wristMotor->SetSelectedSensorPosition(0);

}

void IntakeWrist::kill(){
  wristMotor->Set(ControlMode::PercentOutput, 0);
}