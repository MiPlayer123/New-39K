/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\mikul                                            */
/*    Created:      Thu Mar 10 2022                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// BaseLeftRear         motor         1               
// BaseLeftFront        motor         6               
// BaseRightRear        motor         8               
// BaseRightFront       motor         7               
// Flywheel1            motor         11              
// Intake               motor         4               
// Skills               limit         H               
// STrackO              rotation      16              
// Controller1          controller                    
// Inertial             inertial      5               
// LTrackO              rotation      18              
// RTrackO              rotation      17              
// Flywheel2            motor         15              
// Intake2              motor         19              
// STrack               encoder       A, B            
// RTrack               encoder       C, D            
// LTrack               encoder       E, F            
// Expansion            digital_out   G               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "vex_controller.h"
#include "kalman.h"
#include "control.h"
#include "chasis.h"
#include "buttonSelector.h"
#include "odometry.h"
#include "odom-chassis.h"
#include "drawField.h"

using namespace vex;

task odometryTask;
task drawFieldTask;
task chassisControlTask;

void auton() {

  task odometryTask(positionTracking);
  task chassisControlTask(chassisControl);
  task drawFieldTask(drawField);


  //bool colord = buttons[0].state;
  bool auto1 = buttons[1].state;
  bool auto2 = buttons[2].state;
  bool auto3 = buttons[3].state;
  bool auto4 = buttons[3].state;

  if(is_skills()){
    
  } else if(auto1){
      
  } else if(auto2){
    
  } else if(auto3){
    
  } else if(auto4){
    
  }
  else{
    //turn_absolute_inertial(90);
    //moveRot(1, 50);
    //inertial_drive(12, 50);
    //inertial_drive(12, 30, true);

    //driveTo(30, 30, 2.78, 600, 1.0);
    //waitUntil(runChassisControl == false);
  }
} 

bool locked= false;

void usercontrol() {
  
  // The number of loops we've run
  long ticks = 0;
  
  bool toggle = false;
  bool latch = false;

  while (true) {
    // Get the left and right base speeds from the controller
    //double left_speed = Controller1.Axis3.position();
    //double right_speed = Controller1.Axis2.position();
    int deadzone = 8;

    BaseLeftRear.spin(fwd, 
    + (abs(Controller1.Axis3.position()) > deadzone ? Controller1.Axis3.position() : 0)
    + (abs(Controller1.Axis1.position()) > deadzone ? Controller1.Axis1.position() * 0.5 : 0)
    - (abs(Controller1.Axis4.position()) > deadzone ? Controller1.Axis4.position() : 0), pct);
  BaseLeftFront.spin(fwd, 
    + (abs(Controller1.Axis3.position()) > deadzone ? Controller1.Axis3.position() : 0)
    + (abs(Controller1.Axis1.position()) > deadzone ? Controller1.Axis1.position() * 0.5 : 0)
    + (abs(Controller1.Axis4.position()) > deadzone ? Controller1.Axis4.position() : 0), pct);
  BaseRightRear.spin(fwd, 
    + (abs(Controller1.Axis3.position()) > deadzone ? Controller1.Axis3.position() : 0)
    - (abs(Controller1.Axis1.position()) > deadzone ? Controller1.Axis1.position() * 0.5 : 0)
    + (abs(Controller1.Axis4.position()) > deadzone ? Controller1.Axis4.position() : 0), pct);
  BaseRightFront.spin(fwd, 
    + (abs(Controller1.Axis3.position()) > deadzone ? Controller1.Axis3.position() : 0)
    - (abs(Controller1.Axis1.position()) > deadzone ? Controller1.Axis1.position() * 0.5 : 0)
    - (abs(Controller1.Axis4.position()) > deadzone ? Controller1.Axis4.position() : 0), pct);

    /*
    double forwardBackward = Controller1.Axis2.position();
    double strafe_dir = Controller1.Axis1.position();
    double turn_dir = Controller1.Axis4.position();

    // If the input speed is below our threshold, stop the motors
    if (fabs(forwardBackward) < 5 && fabs(strafe_dir) < 5 && fabs(turn_dir) < 5) {
      BaseLeftRear.stop(brake);
      BaseLeftFront.stop(brake);
      BaseRightRear.stop(brake);
      BaseRightFront.stop(brake);
    }
    // Otherwise spin the motors with the input 
    else {
      BaseRightFront.spin(forward, -forwardBackward + strafe_dir + turn_dir, pct);
      BaseLeftFront.spin(forward, forwardBackward + strafe_dir + turn_dir, pct);
      BaseRightRear.spin(reverse, -forwardBackward - strafe_dir + turn_dir, pct);
      BaseLeftRear.spin(reverse,forwardBackward - strafe_dir + turn_dir, pct);
    }
    */

    // Get the values for the right front buttons
    bool r1_pressing = Controller1.ButtonR1.pressing();
    bool r2_pressing = Controller1.ButtonR2.pressing();
    // Get the values for the left front buttons
    bool l1_pressing = Controller1.ButtonL1.pressing();
    bool l2_pressing = Controller1.ButtonL2.pressing();


    if (toggle){
      spinFlywheel();
    } else {
      stopFlywheel();
    }

    if (l1_pressing) {
      if(!latch){ //flip the toggle one time and set the latch
        toggle = !toggle;
        latch = true;
      }
    } else {
      //Once the BumperA is released then then release the latch too
      latch = false;
    }
    /*
    // If L1 is pressed, 
    if (l1_pressing) {
       spinFlywheel();
    }
    // If L2 is pressed, 
    else if (l2_pressing) {
      //stopFlywheel();
    }
    else{
      stopFlywheel();
    }
    */

    if (l2_pressing ){
      
    }

    if (r1_pressing) {
      spinIntake();
      Intake2.spin(fwd, 100, pct);
    }
    else if (r2_pressing) { 
      Intake.spin(reverse, 100, pct);
      Intake2.spin(reverse, 100, pct);

    }

    else {
      stopIntake();
      Intake2.stop(coast);
    }

    if (Controller1.ButtonUp.pressing()) {
      Expansion.set(true);      
    }
    else if (Controller1.ButtonDown.pressing()) {
      Expansion.set(false);
    } 
    
    if(Controller1.ButtonLeft.pressing()){
      
    } 
    else if (Controller1.ButtonRight.pressing()){
      
    }
    else {
      
    }

    if(Controller1.ButtonA.pressing()){
      
    } 
    else if(Controller1.ButtonY.pressing()){
      
    }

    if(Controller1.ButtonX.pressing()){
      
    } 
    else if (Controller1.ButtonB.pressing()) {
      
    }


    // Increase the tick count
    ticks += 1;
    wait(20.0, msec);
  }
}

competition Competition;

int main() {
  // Initialize vex's internal components
  vexcodeInit();

  //Set encoders to 0
  RTrack.setPosition(0, deg);
  LTrack.setPosition(0, deg);
  STrack.setPosition(0, deg);

  // Calibrate the inertial sensor, and wait for it to finish
  Inertial.calibrate();
  while(Inertial.isCalibrating()) {
    wait(100, msec);
  }

    if(is_skills()){
    THETA_START = M_PI/2; 
    X_START = 0; //19.1
    Y_START = 0; //8.5
  } else{
    THETA_START = 0; //M_PI
    X_START = 0; //19.1
    Y_START = 0; //8.5
  }

  // Print to the screen when we're done calibrating
  Brain.Screen.setCursor(2, 1);
  Brain.Screen.print("Done Calibrating");

  // Initialize our PIDs and rotation tracking thread
  initialize();
  //task mogoHeightTask(mogoHeight);

  BaseLeftRear.setPosition(0, turns);
  BaseRightFront.setPosition(0, turns);
  BaseLeftFront.setPosition(0, turns);
  BaseRightRear.setPosition(0, turns);

  /*
  Brain.Screen.pressed( userTouchCallbackPressed );
  Brain.Screen.released( userTouchCallbackReleased );

  // make nice background
  Brain.Screen.setFillColor( vex::color(0x404040) );
  Brain.Screen.setPenColor( vex::color(0x404040) );
  Brain.Screen.drawRectangle( 0, 0, 480, 120 );
  Brain.Screen.setFillColor( vex::color(0x808080) );
  Brain.Screen.setPenColor( vex::color(0x808080) );
  Brain.Screen.drawRectangle( 0, 120, 480, 120 );

  // initial display
  displayButtonControls( 0, false );

  //display things
  Brain.Screen.setFillColor( vex::color(0xFFFFFF) );
  Brain.Screen.setPenColor( vex::color(0xc11f27));
  Brain.Screen.printAt( 0,  135, "  39K  AQP Technologies Inc." );
  */

  Competition.autonomous(auton);
  Competition.drivercontrol(usercontrol);

  while(true) {
    wait(50, msec);
  }
}
