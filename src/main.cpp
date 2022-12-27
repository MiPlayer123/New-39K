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
// Expansion            digital_out   G               
// AngleAdjust          digital_out   D               
// Optical              optical       7               
// STrackO              rotation      3               
// HyperCarry           limit         A               
// RollerSide           limit         B               
// FarSide              limit         C               
// STrack               encoder       E, F            
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
  //task chassisControlTask(chassisControl);
  task drawFieldTask(drawField);

  //disableBreak();

  //bool colord = buttons[0].state;
  bool auto1 = buttons[1].state;
  bool auto2 = buttons[2].state;
  bool auto3 = buttons[3].state;
  bool auto4 = buttons[3].state;

  if(is_skills()){
    // first roller
    timeCtrl("driveb", .2);
    AutoRoller("red");
    //driveTo(12,137,0);
    //driveTo(10,0,0);
    moveRot(.65, 40);
    turn_absolute_inertial(-45);

    // intake first disc
    spinIntake();
    inertial_drive(27,60);
    turn_absolute_inertial(90);
    stopIntake();

    // second roller
    timeCtrl("driveb", .5);
    AutoRoller("red");
    FwVelocitySet(440, .95);

    // first volley
    spinIntake();
    moveRot(.8,50); //.45
    turn_absolute_inertial(1.5);
    timeCtrl("", .1);
    inertial_drive(53.8,80);
    stopIntake();
    turn_rel_inertial(7);
    volley(440); // shoot

    turn_absolute_inertial(-271); //
    FwVelocitySet(440, .95);
    spinIntake();
    inertial_drive(30, 30);
    inertial_drive(-27, 65);
    turn_absolute_inertial(-350);
    stopIntake();
    volley(440);
    turn_absolute_inertial(140);

    // intake diagonal discs
    spinIntake();
    inertial_drive(35, 60);
    turn_absolute_inertial(45);
    inertial_drive(37, 40);
    wait(250,msec);

    //second volley
    FwVelocitySet(450, .95);
    turn_absolute_inertial(-46);
    stopIntake();
    volley(450); //shoot
    turn_absolute_inertial(45);
    inertial_drive(26,80);
    /*
    // intake 3 stack
    spinIntake();
    inertial_drive(14,10); 
    inertial_drive(12, 60);
    turn_absolute_inertial(-91);

    //third volley
    FwVelocitySet(440, .95);
    inertial_drive(26, 80);
    stopIntake();
    volley(440); // shoot
    turn_absolute_inertial(120);

    // 3stack
    inertial_drive(25,60);
    turn_absolute_inertial(75);
    spinIntake();
    inertial_drive(42, 25);

    // third roller
    turn_absolute_inertial(-90); 
    timeCtrl("driveb", .5);
    AutoRoller("red");
    moveRot(.46,50); //.4

    //Shoot
    FwVelocitySet(430, .95);
    turn_absolute_inertial(-180);
    inertial_drive(48,80);
    volley(430);

    // intake diagonal 2 discs
    inertial_drive(-21, 70);
    turn_absolute_inertial(-90);
    spinIntake();
    inertial_drive(17.5, 60);
    turn_absolute_inertial(-135);
    inertial_drive(37, 40);
    wait(150,msec);

    //fifth volley
    FwVelocitySet(435, .95);
    turn_absolute_inertial(-226); //134
    stopIntake();
    volley(435); //shoot
    turn_absolute_inertial(-133);
    inertial_drive(26,70);
  
    // intake 3 stack
    spinIntake();
    inertial_drive(14,15); 
    inertial_drive(12, 60);
    turn_absolute_inertial(-271);

    //sixth volley
    FwVelocitySet(440, .95);
    inertial_drive(26, 90);
    stopIntake();
    volley(440); // shoot
    
    inertial_drive(-51, 95);
    turn_absolute_inertial(45);
    Expansion.set(true);
    timeCtrl("driveb", .3);
*/
    /*
    inertial_drive(52,90);
    turn_absolute_inertial(90);
    inertial_drive(-16, 80);
    turn_absolute_inertial(45);
    Expansion.set(true);
    */
    
  } else if(auto1 || HyperCarry.pressing()){
    // roller
    FwVelocitySet(575, .98);
    timeCtrl("driveb", .2);
    timeCtrl("intake", .23);
    moveRot(.3,50);

    //shot pre
    //turn_absolute_inertial(-9);
    turnRot(-.2, 40);
    timeCtrl("", 1.1);
    timeCtrl("index", 1.8, 60);

    // intake stack
    turn_absolute_inertial(52); 
    inertial_drive(27, 75);
    FwVelocitySet(535, .95);
    spinIntake();
    inertial_drive(28,40);
    timeCtrl("", .1);

    // second volley
    //turn_absolute_inertial(-41);
    turnRot(-1.82,40);
    timeCtrl("", .15);
    moveRot(.5,40); //.95
    timeCtrl("index", 1.85, 55);
    
  
    // intake diagonal
    turn_absolute_inertial(45.1);
    spinIntake();
    inertial_drive(87,80);
    
    // 2nd roller
    turn_absolute_inertial(-90);
    timeCtrl("driveb",.5);
    timeCtrl("intake", .23);
    FwVelocitySet(0, 0);
    
  } else if(auto2 || RollerSide.pressing()){
    // roller
    //FwVelocitySet(575, .98);
    FwVelocitySet(600, 0.95);
    timeCtrl("driveb", .2);
    timeCtrl("intake", .27);
    moveRot(.3,50);
    turn_absolute_inertial(-35);
    spinIntake();
    moveRot(1,50);
    FwVelocitySet(600, 0.95);
    moveRot(-.5,70);
    turn_absolute_inertial(53); 
    moveRot(2,70);
    turn_absolute_inertial(-12);
    stopIntake();
    moveRot(.5,70);
    timeCtrl("index", 2, 30);
    FwVelocitySet(0, 0.95);
    FwVelocitySet(565, 0.95);
    moveRot(-.5,50);
    turn_absolute_inertial(53); 
    spinIntake();
    inertial_drive(14, 30);
    inertial_drive(22.5, 50);
    turn_absolute_inertial(-32.65);
    stopIntake();
    moveRot(0.5,30);
    timeCtrl("index", 2, 35);
    FwVelocitySet(0, 0.95);
    FwVelocitySet(560, 0.95);
    spinIntake();
    moveRot(1,30);
    moveRot(-1,15);
    stopIntake();
    timeCtrl("index", 1, 100);
    FwVelocitySet(0, 0.95);




    //shot pre
    //turn_absolute_inertial(-9);
    /*
    turnRot(-.2, 40);
    timeCtrl("", 1.1);
    timeCtrl("index", .7, 60);
    timeCtrl("", .3);
    timeCtrl("index", 1, 60);

    // intake stack
    turn_absolute_inertial(53); 
    inertial_drive(27, 75);
    FwVelocitySet(520, .95);
    spinIntake();
    inertial_drive(28,40);
    timeCtrl("", .1);

    // second volley
    //turn_absolute_inertial(-41);
    turnRot(-1.9,40);
    timeCtrl("", .15);
    moveRot(.95,40);
    timeCtrl("index", 1.5, 30); //shoot
    timeCtrl("",.5);
    timeCtrl("index", 1.5, 30);
    FwVelocitySet(0, 0);
    */

  } else if(auto3 || FarSide.pressing()){
    spinIntake();
    FwVelocitySet(535, .95);
    inertial_drive(18, 50);
    turn_absolute_inertial(17); //face goal
    timeCtrl("",.3);
    moveRot(.5,20);
    timeCtrl("index", 1.5, 30); //shoot
    timeCtrl("",.5);
    timeCtrl("index", 2, 30);
    FwVelocitySet(0, 0);

    //roller
    turn_absolute_inertial(-48);
    inertial_drive(-37,50);
    turn_absolute_inertial(0);
    timeCtrl("driveb", .9);
    timeCtrl("intake", .23);

  } else if(auto4){
    
  }
  else{    
    //turnToPoint(12,0);
    driveTo(24,2,0);
    waitTilCompletion();
    turnToPoint(10,-10);
    waitTilCompletion();
    //turnTo(M_PI/2);
    disableBreak();
  }
  
} 

bool locked= false;

void usercontrol() {
  
  // The number of loops we've run
  long ticks = 0;

  int deadzone = 8;

  int flywheelRPM = 460;
  
  // Whether or not the left/right side of the base needs to be stopped
  bool stop_left = true;
  bool stop_right = true;
  
  bool toggle = false;
  bool latch = false;

  while (true) {
    // Get the left and right base speeds from the controller
    double left_speed = Controller1.Axis3.position();
    double right_speed = Controller1.Axis2.position();
    double RT_speed = right_speed*.5;
    double LT_speed = left_speed*.5;

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
    } 
    
    else if(left_speed <0 &right_speed >0 ||left_speed >0 &right_speed <0){
      spin(&BaseLeftRear, LT_speed);
      spin(&BaseLeftFront, LT_speed);
      spin(&BaseLeftMid, LT_speed);
      stop_left = true;
      spin(&BaseRightRear, RT_speed);
      spin(&BaseRightFront, RT_speed);
      spin(&BaseRightMid, RT_speed);
      stop_right = true;
    }

    else {
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
    X_START = 10; //19.1
    Y_START = 137; //8.5
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
