#include "subsystems/baseDrive.h"

baseDrive::baseDrive() : Subsystem("ExampleSubsystem") {}

void baseDrive::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
}

void baseDrive::xyJoystickControl(double inputX1, double inputY, double inputX2){

//set point is vector of linear motion across x and y axis.
//imagine the floor as a 2D argand plane.
//rotation should be an added modification of power.
//Marc, this would be a lot less exhaustive if it had talons...
// -ZJ

//first we must calculate the geometrical distance of the diagonals across the chassis.
//first person to figure out why gets $20 hard cash -ZJ

double L = 0; //FIND LENGTH OF CHASSIS (WHEEL CONTACT TO WHEEL CONTACT)
double W = 0; //SAME FOR WIDTH
double diag = sqrt((L*L) + (W*W));

double a = inputX1 - inputX2 * (L / diag);
double b = inputX1 + inputX2 * (L / diag); //hint on the money problem, use a calculator to conceptualize.
double c = inputY - inputX2 * (W / diag); //what might happen when you divide the components of a quadralateral by the constituent diagonal :thinking:
double d = inputY + inputX2 * (W / diag); //the input y's might need to be inverted<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>><><><><

//SENSOR IN EACH MOTOR DETECTS 7 PULSES PER REVOLUTION

//Now to calculate the translation vector, or the speed vector or whatever
double translateFrontRight = sqrt((a*a) + (d*d));
double translateFrontLeft = sqrt((b*b) + (c*c));
double translateBackRight = sqrt((a*a) + (d*d)); 
double translateBackLeft = sqrt((b*b) + (c*c));

//THE BEAST - 4 PIDS CONTROLLING LINEAR MOTION IN TWO DEMENTIONS
//FR
swerveSetpointFR = (atan2(b,d)/ M_PI)*180; //$5 to someone who can figure out why and how.

swerveCurrFR = (eFrontRight->Get()/7)*360; //gives out current angle in degrees because baby mode
swerveErrFR = swerveCurrFR - swerveSetpointFR;
swerveDerivativeFR = swerveErrFR - swervePrevErrFR;
swervePrevErrFR = swerveErrFR;
swerveCorrectionFR = kPSwerve*swerveErrFR + kDSwerve*swerveErrFR; //Sent correction to pivot motor on the front right

//FL
swerveSetpointFL = (atan2(b, c)/ M_PI)*180;

swerveCurrFL = (eFrontLeft->Get()/7)*360;
swerveErrFL = swerveCurrFL - swerveSetpointFL;
swerveDerivativeFL = swerveErrFL - swervePrevErrFL;
swervePrevErrFL = swerveErrFL;
swerveCorrectionFL = kPSwerve*swerveErrFL + kDSwerve*swerveErrFL; //Sent correction to pivot motor on the front left

//BR
swerveSetpointBR = (atan2(a, d)/ M_PI)*180;

swerveCurrBR = (eBackRight->Get()/7)*360;
swerveErrBR = swerveCurrBR - swerveSetpointBR;
swerveDerivativeBR = swerveErrBR - swervePrevErrFR;
swervePrevErrBR = swerveErrBR;
swerveCorrectionBR = kPSwerve*swerveErrBR + kDSwerve*swerveErrBR; //Sent correction to pivot motor on the back right

//BL
swerveSetpointBL = (atan2(a, c)/ M_PI)*180;

swerveCurrBL = (eBackLeft->Get()/7)*360;
swerveErrBL = swerveCurrBL - swerveSetpointBL;
swerveDerivativeBL = swerveErrBL - swervePrevErrBL;
swervePrevErrBL = swerveErrBL;
swerveCorrectionBL = kPSwerve*swerveErrBL + kDSwerve*swerveErrBL; //Sent correction to pivot motor on the back left

//SEND VALUES TO MOTORS
pivotFrontRight->Set(swerveCorrectionFR +  translateFrontRight); //without the speed vector(these are angular vectors) the drive will follow but it cant do anything.
pivotFrontLeft->Set(swerveCorrectionFL + translateFrontLeft); //I think it would be able to spin but im not exactly sure.
pivotBackRight->Set(swerveCorrectionBR + translateBackRight); 
pivotBackLeft->Set(swerveCorrectionBL + translateBackLeft);

}
