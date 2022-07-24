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
// BaseLeftRear         motor         14              
// BaseLeftFront        motor         13              
// BaseRightRear        motor         20              
// BaseRightFront       motor         2               
// Flywheel1            motor         1               
// Intake               motor         18              
// Skills               limit         H               
// Index                digital_out   A               
// STrack               rotation      7               
// Controller1          controller                    
// Inertial             inertial      19              
// LTrack               rotation      9               
// RTrack               rotation      16              
// Flywheel2            motor         12              
// Turret               motor         10              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "vex_controller.h"
#include "kalman.h"
#include "control.h"
#include "chasis.h"
#include "buttonSelector.h"

using namespace vex;
void swinging(double leftPower, double rightPower, double gogoAngle){
  BaseLeftFront.spin(fwd, 12*leftPower/100, volt);  
  BaseLeftRear.spin(fwd, 12*leftPower/100, volt); 
  BaseRightFront.spin(fwd, 12*rightPower/100, volt);  
  BaseRightRear.spin(fwd, 12*rightPower/100, volt); 
  if(get_rotation() > gogoAngle){
    while(get_rotation() > (gogoAngle+4)){
      wait(5,msec); 
    }
  }
  else{
    while((gogoAngle-10) > get_rotation()){
      wait(5,msec);
    }
  }
  brake_unchecked(); 
}
void auton() {

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

  }
} 

bool locked= false;

void usercontrol() {
  
  // Whether or not the left/right side of the base needs to be stopped
  bool stop_left = true;
  bool stop_right = true;
  // The number of loops we've run
  long ticks = 0;

  //Intake stat
  //bool intakeStat = false;

  while (true) {
    // Get the left and right base speeds from the controller
    //double left_speed = Controller1.Axis3.position();
    //double right_speed = Controller1.Axis2.position();

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

    // Get the values for the right front buttons
    bool r1_pressing = Controller1.ButtonR1.pressing();
    bool r2_pressing = Controller1.ButtonR2.pressing();
    // Get the values for the left front buttons
    bool l1_pressing = Controller1.ButtonL1.pressing();
    bool l2_pressing = Controller1.ButtonL2.pressing();


    // If L1 is pressed, 
    if (l1_pressing) {
       
      }
    // If L2 is pressed, 
    else if (l2_pressing) {
      
    }
    else{}

    if (r1_pressing) {
      
    }
    else if (r2_pressing) { 
      
    }

    else {
     
    }

    if (Controller1.ButtonUp.pressing()) {
      
    }
    else if (Controller1.ButtonDown.pressing()) {
      
    } 
    else if(Controller1.ButtonLeft.pressing()){
      //Toggle lock
     
    } 
    else if (Controller1.ButtonRight.pressing()){
      //Mogo to ring height
    }
    else {}

    if(Controller1.ButtonA.pressing()){
      
    } 
    else if(Controller1.ButtonY.pressing()){
      
    }

    if(Controller1.ButtonX.pressing()){
      
    } else if (Controller1.ButtonB.pressing()) {
      
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
  //ROdom.setPosition(0, deg);
  //LOdom.setPosition(0, deg);
  //SOdom.setPosition(0, deg);

  // Calibrate the inertial sensor, and wait for it to finish
  Inertial.calibrate();
  while(Inertial.isCalibrating()) {
    wait(100, msec);
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
