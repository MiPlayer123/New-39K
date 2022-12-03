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
// BaseLeftRear         motor         20              
// BaseLeftFront        motor         1               
// BaseRightRear        motor         19              
// BaseRightFront       motor         6               
// BaseLeftMid          motor         11              
// BaseRightMid         motor         8               
// Skills               limit         H               
// Controller1          controller                    
// Inertial             inertial      13              
// LTrack               rotation      18              
// RTrack               rotation      17              
// Flywheel             motor         15              
// Intake               motor         2               
// STrack               encoder       C, D            
// Expansion            digital_out   G               
// AngleAdjust          digital_out   F               
// Optical              optical       7               
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

  //task odometryTask(positionTracking);
  //task chassisControlTask(chassisControl);
  //task drawFieldTask(drawField);


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
    // hypercarry

    timeCtrl("driveb", .2);
    timeCtrl("intake", .2);
    moveRot(.3,50);
    turn_absolute_inertial(45);
    FwVelocitySet(570, .95);
    inertial_drive(30, 70);
    spinIntake();
    inertial_drive(23,30);
    timeCtrl("", .25);
    turn_absolute_inertial(-40);
    timeCtrl("", .4);
    moveRot(.4,80);
    Intake.spin(reverse,70,pct);
    timeCtrl("", 2.5);
    stopIntake();
    moveRot(-.4,80);
    turn_absolute_inertial(47);
    spinIntake();
    FwVelocitySet(600, .95);
    inertial_drive(60,70); 
    timeCtrl("", .1);
    turn_absolute_inertial(-77);
    moveRot(.5,80);
    timeCtrl("",.5);
    Intake.spin(reverse,80,pct);
    timeCtrl("", 2);
    stopIntake();
    turn_absolute_inertial(-145);
    inertial_drive(-28, 80);
    turn_absolute_inertial(-90);
    timeCtrl("driveb",.8);
    timeCtrl("intake", .2);
    FwVelocitySet(0, .95);

    

    /*
    timeCtrl("driveb", .2);
    timeCtrl("intake", .25);
    moveRot(.4,50);
    turn_absolute_inertial(45);
    FwVelocitySet(590, .95);
    inertial_drive(60,50);
    turn_absolute_inertial(-40);
    moveRot(.4,80);
    timeCtrl("index", 2.5);
    turn_absolute_inertial(45);
    */


    
  


    /*
    FwVelocitySet(600, .95);
    timeCtrl("driveb", .2);
    timeCtrl("intake", .25);
    moveRot(.3,50);
    turn_absolute_inertial(-8.5);
    volley(550,2.5);
    moveRot(.25,50);
    turn_absolute_inertial(55);
    spinIndex();
    inertial_drive(30,60);
    spinIntake();
    inertial_drive(10.5,10);
    inertial_drive(10.5,60);
    turn_absolute_inertial(-40);
    FwVelocitySet(600, .95);
    moveRot(.8,40);
    stopIntake();
*/



    /*
    timeCtrl("driveb", .2);
    //first roller
    AutoRoller("red");
    moveRot(.65, 40);
    turn_absolute_inertial(-45);
    spinIntake();
    moveRot(3.85,40);
    turn_absolute_inertial(90);
    stopIntake();
    timeCtrl("driveb", .38);
    AutoRoller("red");
    //second roller
    moveRot(.30,50);
    turn_absolute_inertial(-0);
    inertial_drive(55,60);
    FwVelocitySet(450, .95);
    turn_absolute_inertial(8);
    volley(430);
    //first volley
    turn_absolute_inertial(-0);
    inertial_drive(-29, 50);
    turn_absolute_inertial(90);
    spinIntake();
    inertial_drive(17.5, 40);
    turn_absolute_inertial(47);
    inertial_drive(37, 40);
    wait(1000,msec);
    FwVelocitySet(480, .95);
    turn_absolute_inertial(-45);
    stopIntake();
    volley(480);
    //second volley
    turn_absolute_inertial(45);
    inertial_drive(26,70);
    spinIntake();
    inertial_drive(24,10);
    inertial_drive(10, 60);
    turn_absolute_inertial(-90);
    stopIntake();
    FwVelocitySet(430, .95);
    inertial_drive(36, 50);
    volley(430);
    timeCtrl("driveb", 3);
    inertial_drive(10, 60);
    turn_absolute_inertial(-45); 
    */

    //zExpansion.set(true);

    //third volley
    //Later
    //inertial_drive(, 60)

    
  }
} 

bool locked= false;

void usercontrol() {
  
  // The number of loops we've run
  long ticks = 0;

  int deadzone = 8;

  int flywheelRPM = 450;

  // Whether or not the left/right side of the base needs to be stopped
  bool stop_left = true;
  bool stop_right = true;
  
  bool toggle = false;
  bool latch = false;

  while (true) {
    // Get the left and right base speeds from the controller
    double left_speed = Controller1.Axis3.position();
    double right_speed = Controller1.Axis2.position();

    // If the input speed is below our threshold, stop the motors
    if ((left_speed < deadzone && left_speed > -deadzone)) {
      // This condition only calls the stop instruction once        
      BaseLeftRear.stop(brake);
      BaseLeftFront.stop(brake);
      BaseLeftMid.stop(brake);
      stop_left = false;
    }
    // Otherwise spin the motors with the input 
    else {
      /*
      if (left_speed <0){
        left_speed = -((pow(fabs(left_speed), 4 *.4))/pow(100, (4*.4)-1));
      }
      else {
        left_speed = ((pow(fabs(left_speed), 4 *.4))/pow(100, (4*.4)-1)); 
      }
      if((left_speed*right_speed <0)){
        left_speed = .7*left_speed; 
      }
      */
      spin(&BaseLeftRear, left_speed);
      spin(&BaseLeftFront, left_speed);
      spin(&BaseLeftMid, left_speed);
      stop_left = true;
    }

    // This is equivalent to the code above
    if ((right_speed < deadzone && right_speed > -deadzone)) {
      
      BaseRightRear.stop(brake);
      BaseRightFront.stop(brake);
      BaseRightMid.stop(brake);
      stop_right = false;
    } else {
      /*
      if (right_speed <0){
        right_speed = -((pow(fabs(right_speed), 4 *.4))/pow(100, (4*.4)-1));
      }
      else {
        right_speed = ((pow(fabs(right_speed), 4 *.4))/pow(100, (4*.4)-1)); 
      }
      if((right_speed*left_speed) <0){
        right_speed = .7*right_speed; 
      */
      spin(&BaseRightRear, right_speed);
      spin(&BaseRightFront, right_speed);
      spin(&BaseRightMid, right_speed);
      stop_right = true;
    }

    // Get the values for the right front buttons
    bool r1_pressing = Controller1.ButtonR1.pressing();
    bool r2_pressing = Controller1.ButtonR2.pressing();
    // Get the values for the left front buttons
    bool l1_pressing = Controller1.ButtonL1.pressing();
    bool l2_pressing = Controller1.ButtonL2.pressing();


    if (toggle && latch){
      FwVelocitySet( flywheelRPM, .95 );
    } else if(!toggle) {
      FwVelocitySet( 0, 0 );
    }

    if (l1_pressing) {
      if(!latch){ //flip the toggle one time and set the latch
        toggle = !toggle;
        latch = true;
      }
    } else {
      //Once the Bumper is released then then release the latch too
      latch = false;
    }

    if (l2_pressing ){}

    if (r1_pressing) {
      spinIndex();
    }
    else if (r2_pressing) { 
      spinIntake();
    }
    else {
      stopIntake();
    }

    if (Controller1.ButtonUp.pressing()) {
      AngleAdjust.set(true);      
    }
    else if (Controller1.ButtonDown.pressing()) {
      AngleAdjust.set(false);
    } 
    
    if(Controller1.ButtonLeft.pressing()){
      flywheelRPM-=50;
    } 
    else if (Controller1.ButtonRight.pressing()){
      flywheelRPM+=50;
      if(flywheelRPM>=600)
        flywheelRPM=600;
    }
    else {}

    if(Controller1.ButtonA.pressing()){
      AutoRoller("red");
    } 
    else if(Controller1.ButtonY.pressing()){}

    if(Controller1.ButtonX.pressing()){
      Expansion.set(true);
    } 
    else if (Controller1.ButtonB.pressing()){
      Expansion.set(false);
    }

    double relRPM = Flywheel.velocity(rpm);
    double actualRPM = relRPM*6;

    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("X: %.1lf Y: %.1lf Theta: %.1lf", 0.0,0.0, get_rotation());
    Brain.Screen.setCursor(5, 1);
    Brain.Screen.print("RPM: %.1lf Actual RPM: %.1lf", relRPM, actualRPM);
    Brain.Screen.setCursor(6, 1);
    Brain.Screen.print("Base Temp %.0lf Flywheel Temp: %.0lf Intake temp: %.0lf", 
      BaseRightMid.temperature(celsius), Flywheel.temperature(celsius), Intake.temperature(celsius));
    
    Controller1.Screen.setCursor(3, 1);
    Controller1.Screen.print("Set: %.0f Actual: %.0lf", (float)flywheelRPM, relRPM); 

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

  BaseLeftRear.setPosition(0, turns);
  BaseRightFront.setPosition(0, turns);
  BaseRightMid.setPosition(0, turns);
  BaseLeftMid.setPosition(0, turns);
  BaseLeftFront.setPosition(0, turns);
  BaseRightRear.setPosition(0, turns);

  Flywheel.setPosition(0, deg);

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
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("Done Calibrating");

  // Initialize our PIDs and rotation tracking thread
  initialize();

  task fwControl(FwControlTask); //Flywheel control

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
