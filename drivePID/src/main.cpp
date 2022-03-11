/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\CibolaA                                          */
/*    Created:      Tue Dec 07 2021                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
using namespace vex;

motor_group LeftDriveSmart = motor_group(FrontLeft, BackLeft);

motor_group RightDriveSmart = motor_group(FrontRight, BackRight);

smartdrive Drivetrain = smartdrive(LeftDriveSmart, RightDriveSmart, TurnGyroSmart, 319.19, 320, 40, mm, 1);



//// drive pid//////
int targetDistance = 0;

//gains

double dkI = 0.04;

double dkD = 0.03;

double dkP = 0.15;

//drive threshold for integral (2 inches)



double tickDistance = 0;

//drivetrain wheel diameter in inches
double wheelDiameter = 4;

//pi
double pi = 3.14159265358979;

// total ticks in encoder
double eTicks = 900;

//drive threshold for integral 
// needs to be tuned
//lower than 10 doesn't do anything so at least 10 but will have to test
double driveThreshold = 9;


//function to say drive x distance in ft
void driveTo (double targetDistance) {
  //reset motor encoders so they are all zero
  //this way the ticks match up without worrying about turning
  BackLeft.resetRotation();
  BackRight.resetRotation();
  FrontLeft.resetRotation();
  FrontRight.resetRotation();

//declaration of local variables
double integral = 0;
  double error = 0;
double derivative = 0;
double prevError =0;
//converting target distance into ticks
tickDistance = fabs(targetDistance / (wheelDiameter * pi) *eTicks);
double wheelConstant = wheelDiameter * pi * 1;

//while loop
//checks desired distance against sensor of current distance driven
// 10 allows us to have a threshold for ticks so if its close enough it stops but check math in case 10 is too high
//Check if threshold and this threshold need to be same
     while (fabs(tickDistance) > (fabs(FrontRight.rotation(rotationUnits::deg)) * 2.5 / wheelConstant) || (fabs(tickDistance) - (fabs(FrontRight.rotation(rotationUnits::deg)) * 2.5 / wheelConstant) > 10)) {
//error is tick distance - sensor
    error = tickDistance - (fabs(FrontRight.rotation(rotationUnits::deg)) * 2.5 / wheelConstant);
//assign derivative 
    derivative = error - prevError;
    //assign previous error as the error before
    prevError = error;

//if error is less than threshold and error is not 0 add integral + error to integral
if (fabs(error) < driveThreshold && error != 0) {
  integral += error;
  //else nothing
} else {
  integral = 0;
}
//end of if

//declare and assign powerdrive (I.E. velocity control PID)
   double powerDrive = (error * dkP) + (derivative * dkD) + (integral * dkI);

   //if the distance is positive drive forward
    if(targetDistance > 0) {
    LeftDriveSmart.spin(forward,powerDrive,voltageUnits::volt);
      RightDriveSmart.spin(forward,powerDrive,voltageUnits::volt);
    }
    //end of positive drive if

    //if the distance is negative drive reverse
    if (targetDistance < 0) {
        LeftDriveSmart.spin(reverse,powerDrive,voltageUnits::volt);
      RightDriveSmart.spin(reverse,powerDrive,voltageUnits::volt);
    }
    //end of negative drive if

    this_thread::sleep_for(15);
    }//end of while loop
    
    //tell motors to stop if target is achieved
LeftDriveSmart.stop();
RightDriveSmart.stop();

//print data and assign last values
    error = tickDistance - (fabs(FrontRight.rotation(rotationUnits::deg)) * 2.5 / wheelConstant);
    derivative = error - prevError;
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.setCursor(1, 13);
  Controller1.Screen.newLine();
  Controller1.Screen.print("error: %.5f", error);
  Controller1.Screen.newLine();
  Controller1.Screen.print("derivative %.5f", error);
  Controller1.Screen.newLine();
  }//end of function



int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  //Controller1.ButtonA.pressed(PIDT);
  Controller1.Screen.print(FrontLeft.temperature());
}
