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
// OpticalLeft          optical       7               
// STrackO              rotation      3               
// HyperCarry           limit         A               
// RollerSide           limit         B               
// FarSide              limit         C               
// OpticalRight         optical       5               
// SideExpansion        digital_out   E               
// STrack               rotation      4               
// ThreeStack           digital_out   F               
// Inertial2            inertial      16              
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
//task drawFieldTask;
//task chassisControlTask;

void auton() {

  //task odometryTask(positionTracking);
  //task chassisControlTask(chassisControl);
  //task drawFieldTask(drawField);

  disableBreak();

  //bool colord = buttons[0].state;
  bool auto1 = buttons[1].state;
  bool auto2 = buttons[2].state;
  bool auto3 = buttons[3].state;
  bool auto4 = buttons[3].state;

  if(is_skills()){
// first roller
    OpticalLeft.setLightPower(40, percent); 
    OpticalRight.setLightPower(40, percent);
    spinIntake();
    timeCtrl("driveb", .10);
    AutoRoller("red", 1);
    //driveTo(12,137,0);
    //driveTo(10,0,0);
    moveRot(.65, 40);
    turn_absolute_inertial(-45);
    // intake first disc
    spinIntake();
    inertial_drive(24.5,85);
    turn_absolute_inertial(90.5);

    // second roller
    timeCtrl("driveb", .3);
    AutoRoller("red", 2);
    setFlywheelVel(470);

    // first volley
    spinIntake();
    moveRot(.7,50); //.45
    turn_absolute_inertial(0.0);
    inertial_drive(45.5,95);
    stopIntake();
    //turn_absolute_inertial(4.0);
    volley(470); // shoot
   // turn_absolute_inertial(1.0);
    turn_absolute_inertial(137.5);//-214.5
    
    // intake diagonal discs
    spinIntake();
    setFlywheelVel(470);
    inertial_drive(27.5, 80);
    turn_absolute_inertial(45);
    inertial_drive(36.5, 70);
    wait(150,msec);

    //second volley
    setFlywheelVel(472);
    turn_absolute_inertial(-47);//-46
    longVolley(472); //shoot
    spinIndex();
    
    inertial_drive(-18.5,50);
    stopIntake();
    turn_absolute_inertial(45);
    spinIntake();
    inertial_drive(32, 70);
    turn_absolute_inertial(0.0);
    inertial_drive(30, 70);
    turn_absolute_inertial(90);
    inertial_drive(11, 60);
    turn_absolute_inertial(180.0);
    //third roller
    timeCtrl("driveb", .6);
    AutoRoller("red", 1);
    moveRot(.65, 40);
    turn_absolute_inertial(135);
    spinIntake();
    inertial_drive(29,85);
    turn_absolute_inertial(270.0);

    // fourth roller
    timeCtrl("driveb", .4);
    AutoRoller("red", 2);
    setFlywheelVel(460);
    
    spinIntake();
    moveRot(.7,50); //.45
    turn_absolute_inertial(180.0);
    inertial_drive(45.5,92);
    stopIntake();
    //third volley
    volley(460); // shoot
  
    turn_absolute_inertial(319);//-214.5
    
    // intake diagonal discs
    spinIntake();
    setFlywheelVel(470);
    inertial_drive(28.5, 80);
    turn_absolute_inertial(225);
    inertial_drive(35, 70);
    wait(150,msec);

    //fourth volley
    turn_absolute_inertial(132.2);//-46
    stopIntake();
    longVolley(471); //shoot
    spinIndex();

    inertial_drive(-16.5, 50);
    stopIntake();
    turn_absolute_inertial(225);
    spinIntake();
    setFlywheelVel(475);
    inertial_drive(34, 90);
    inertial_drive(12, 70);

    turn_absolute_inertial(-13+360);
    inertial_drive(40,95);
    volley(475);
    spinIndex();

    turn_absolute_inertial(138.8);
    stopIntake();
    spinIntake();
    setFlywheelVel(470);
    inertial_drive(49, 90);
    inertial_drive(30, 80);
    turn_absolute_inertial(80);
    volley(470);//*/
    /*
    turn_absolute_inertial(0.0);
    spinIntake();
    inertial_drive(94, 95);
    inertial_drive(10, 85);
    turn_absolute_inertial(-90.0);
    volley(470);
    inertial_drive(-45, 95);
    turn_absolute_inertial(-135);*/
    turn_absolute_inertial(91.5);
    inertial_drive(-61, 98);
    turn_absolute_inertial(44.5);
    Expansion.set(true);
  
  } else if(auto1 || HyperCarry.pressing()){
    // roller
    
    setFlywheelVel(585); 
    timeCtrl("driveb", .1);
    timeCtrl("intake", .4);
    moveRot(.53,60);

    turn_absolute_inertial(52.2);
    spinIntake();
    inertial_drive(27, 75);
    timeCtrl("", .1);
    turn_absolute_inertial(-23.85);
    //wait(500, msec);
    wait(200, msec);
    longVolley(576);

    // second volley
    setFlywheelVel(557);
    turn_absolute_inertial(49.2);
    spinIntake();
    inertial_drive(41, 75);
    turn_absolute_inertial(-47.2);
    wait(100, msec);
    longVolley(540);
    turn_absolute_inertial(43.0);
    setFlywheelVel(0);
    spinIntake();
    inertial_drive(65, 85);
    
    //2nd roller
    turnRot(-1.83,97);
    //turn_absolute_inertial(-90);

    timeCtrl("driveb",.4);
    timeCtrl("intake", .42);
    
  } else if(auto2 || RollerSide.pressing()){
    // roller
    setFlywheelVel(588); // 85
    timeCtrl("driveb", .1);
    timeCtrl("intake", .32);
    inertial_drive(10.05, 45);
    wait(150, msec);
    turn_absolute_inertial(-13.95);


    ThreeStack.set(true);
    moveRot(.49,5);
    timeCtrl("index", .30, 100); 
    timeCtrl("intake",.48);
    timeCtrl("index", .30, 100); 
    ThreeStack.set(false);
    spinIntake();
    timeCtrl("", 2.5);
    
    longVolley(574);

    moveRot(-1.4,30);
    setFlywheelVel(570); //570 35
    turn_absolute_inertial(49.0); 
    spinIntake();
    inertial_drive(17, 95);
    inertial_drive(22.5, 40);
    //Shoot 3
    turn_absolute_inertial(-36.2);
    wait(200, msec);
    longVolley(545);
    stopIntake();
    setFlywheelVel(0);
    //Get Midline
    /*
    moveRot(1.35,30);
    moveRot(-.8,15);
    timeCtrl("", .3);
    timeCtrl("index", .5, 100);
    FwVelocitySet(0,0);
    */

  } else if(auto3 || FarSide.pressing()){
    // intake 1
    setFlywheelVel(608);
    inertial_drive(-20, 60);

    turn_absolute_inertial(90);
    timeCtrl("driveb", .29); //.27
    timeCtrl("intake", .41);
    moveRot(.5,65);
    turn_absolute_inertial(43);
    spinIntake();
    inertial_drive(27,85);

    // 1st volley
    turn_absolute_inertial(110.0);
    moveRot(.6,35);
    longVolley(581);
    setFlywheelVel(586);
    turn_absolute_inertial(115);

    // intake 
    spinIntake();
    moveRot(.9, 35);
    moveRot(-1.5, 35);
    turn_absolute_inertial(46.7);
    inertial_drive(20,85);

    //inertial_drive(-19, 70);

    // 2nd volley
    turn_absolute_inertial(136); //120.3
    moveRot(.8,45); 
    turn_absolute_inertial(121.3);
    longVolley(577);

    //setFlywheelVel(0);
        /*
    timeCtrl("",.1);
    spinIntake();
    moveRot(1,55);
    moveRot(-1,20);
    //timeCtrl("index", 1, 100);
    FwVelocitySet(0, 0);
    */
  } else if(auto4){
    
  }
  else{    
    //inertial_drive(60,80);
    //turn_absolute_inertial(90);
    setFlywheelVel(580);
    wait(2000,msec);
    longVolley(580);
    //turnToPoint(12,5);
    //waitTilCompletion();
    //driveTo(12,0);
    //waitTilCompletion();
    //turnTo(M_PI/2);
    //waitTilCompletion();
    //turnTo(M_PI/2);
    //disableBreak();
    //turn_absolute_inertial(90);
    //moveToPoint(20, 20, 60);
  }
  
} 

bool locked= false;

void usercontrol() {
  
  // The number of loops we've run
  long ticks = 0;

  int deadzone = 8;

  int defaultRPM = 475; //470
  int angleRpm = 465;
  int flywheelRPM = defaultRPM;
  
  //bool expanded = false;
  // Whether or not the left/right side of the base needs to be stopped
  bool stop_left = true;
  bool stop_right = true;
  
  bool toggle = false;
  bool latch = false;

 // drawFieldTask.suspend();
 // chassisControlTask.suspend();
  Brain.Screen.clearScreen();

  while (true) {
    // Get the left and right base speeds from the controller
    double left_speed = Controller1.Axis3.position();
    double right_speed = Controller1.Axis2.position();
    double RT_speed = right_speed*1;
    double LT_speed = left_speed*1;

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
      //FwVelocitySet( flywheelRPM, .95 );
      setFlywheelVel(flywheelRPM);
    } else if(!toggle) {
      //FwVelocitySet( 0, 0 );
      setFlywheelVel(0);
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

    if (l2_pressing ){
        AngleAdjust.set(true);
        flywheelRPM = angleRpm;
        setFlywheelVel(flywheelRPM);
        toggle = !toggle;
        latch = true;
    }

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
      flywheelRPM=angleRpm;
      FwVelocitySet( flywheelRPM, .95 );
      setFlywheelVel(flywheelRPM);
      //inertial_drive(48, 97);    
    }
    else if (Controller1.ButtonDown.pressing()) {
      AngleAdjust.set(false);
      flywheelRPM = defaultRPM;
      FwVelocitySet( flywheelRPM, .95 );
      setFlywheelVel(flywheelRPM);
      //inertial_drive(-24, 97);
      } 
    
    if(Controller1.ButtonLeft.pressing()){
      flywheelRPM-=50;
      FwVelocitySet( flywheelRPM, .95 );
      setFlywheelVel(flywheelRPM);
      //turn_rel_inertial(-90);
    } 
    else if (Controller1.ButtonRight.pressing()){
      flywheelRPM+=50;
      if(flywheelRPM>=600)
        flywheelRPM=600;
      FwVelocitySet( flywheelRPM, .95 );
      setFlywheelVel(flywheelRPM);
      //turn_rel_inertial(90);
    }
    else {}

    if(Controller1.ButtonA.pressing()){} 
    if(Controller1.ButtonY.pressing()){
      SideExpansion.set(true);
    }

    if(Controller1.ButtonX.pressing()){
      Expansion.set(true);
      /*if (expanded){
        SideExpansion.set(true);
      }
      expanded = true;*/
    } 
    else if (Controller1.ButtonB.pressing()){
      Expansion.set(false);
      SideExpansion.set(false);
    }

    double relRPM = Flywheel.velocity(rpm);
    double actualRPM = relRPM*6;

    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("X: %.1lf Y: %.1lf Theta: %.1lf", xPosGlobal,yPosGlobal, get_rotation());
    Brain.Screen.setCursor(5, 1);
    Brain.Screen.print("RPM: %.1lf Actual RPM: %.1lf", relRPM, actualRPM);
    //Brain.Screen.drawImageFromFile("logo.png", 1,1);
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Base Temp %.0lf Flywheel Temp: %.0lf Intake temp: %.0lf", 
    BaseRightMid.temperature(celsius), Flywheel.temperature(celsius), Intake.temperature(celsius));

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
  Inertial2.calibrate();
  while(Inertial.isCalibrating() || Inertial2.isCalibrating()) {
    wait(100, msec);
  }

  if(is_skills()){
    THETA_START = 0; 
    X_START = 137; //19.1
    Y_START = 10; //8.5
    setPos(137, 10);
  } else{
    THETA_START = 0.0; //M_PI
    X_START = 10.0; //19.1
    Y_START = 10.0; //8.5
    setPos(10, 10);
  }

  // Print to the screen when we're done calibrating
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("Done Calibrating");

  // Initialize our PIDs and rotation tracking thread
  initialize();

  task FC(flywheelControl); //the kaavin
  //task flyctrl(flyCtrl); //the ryan
  //task fwControl(FwControlTask); //Flywheel control
  task odometryTask(positionTracking);
  

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
