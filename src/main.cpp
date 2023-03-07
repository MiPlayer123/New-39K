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
    spinIntake();
    OpticalLeft.setLightPower(40, percent); 
    OpticalRight.setLightPower(40, percent);
    setFlywheelVel(478);//450
    timeCtrl("driveb", .12);
    AutoRoller("red", 1);
    //driveTo(12,137,0);
    //driveTo(10,0,0);
    moveRot(.65, 40);
    turn_absolute_inertial(-45);
    // intake first disc
    spinIntake();
    inertial_drive(25,85);
    turn_absolute_inertial(90.5);

    // second roller
    timeCtrl("driveb", .35);
    AutoRoller("red", 2);

    // first volley
    spinIntake();
    moveRot(.7,45); //.65
    turn_absolute_inertial(1.1);
    //inertial_drive(44,87);
    inertial_drive(25, 87);
    stopIntake();
    turnRot(.108,20);
    //volley(462); // shoot
    //turn_absolute_inertial(135);//-214.5
    longVolley(.17);
    turn_absolute_inertial(90);
    
    // intake diagonal discs
    spinIntake();
    setFlywheelVel(469);
    //inertial_drive(27.5, 80);
    inertial_drive(20, 80);
    turn_absolute_inertial(45);
    inertial_drive(34, 60);
    wait(150,msec);

    //second volley
    turn_absolute_inertial(-47);//-46
    setFlywheelVel(466);
    longVolley(.17); //shoot
    
    inertial_drive(-16.4,50);
    turn_absolute_inertial(45);
    spinIntake();
    inertial_drive(32.5, 60);
    turn_absolute_inertial(0.0);
    inertial_drive(30, 60);
    turn_absolute_inertial(90);
    inertial_drive(11, 60);
    turn_absolute_inertial(180.0);
    //third roller
    timeCtrl("driveb", .6);
    AutoRoller("red", 1);
    moveRot(.65, 40);
    turn_absolute_inertial(135.5);
    spinIntake();
    inertial_drive(28,85);
    turn_absolute_inertial(270.0);

    // fourth roller
    timeCtrl("driveb", .4);
    AutoRoller("red", 2);
    setFlywheelVel(477);

    spinIntake();
    moveRot(.7,50); //.45
    turn_absolute_inertial(181.0);
    //inertial_drive(43,90);
    inertial_drive(25, 90);
    stopIntake();
    turnRot(.08,20);
    longVolley(.16);
    turn_absolute_inertial(270);
    //turnRot(.09,20);
    //stopIntake();
    //volley(462); // shoot
    //turn_absolute_inertial(319);//-214.5
    
    // intake diagonal discs
    spinIntake();
    setFlywheelVel(464);
    //inertial_drive(29, 80);
    inertial_drive(22, 80);
    turn_absolute_inertial(225);
    inertial_drive(34, 60);
    wait(50,msec);

    //second volley
    turn_absolute_inertial(133);//-46
    stopIntake();
    setFlywheelVel(477);
    longVolley(.14); //shoot Diagnonal
    spinIndex();

    inertial_drive(-16.5, 50);
    turn_absolute_inertial(225);
    spinIntake(); //inake mid
    inertial_drive(34, 70);
    inertial_drive(12, 80);

    turn_absolute_inertial(349);
    inertial_drive(42,95);
    volley(463);//shoot 

    turn_absolute_inertial(147);
    setFlywheelVel(465);
    spinIntake(); //intake 3 close
    inertial_drive(49, 90); 
    inertial_drive(32, 80); //30
    turn_absolute_inertial(78);
    volley(462); //shoot 3

    turn_absolute_inertial(5.3);
    setFlywheelVel(467);
    spinIntake();//inake far 3
    inertial_drive(92, 95);
    inertial_drive(22, 85);
    turn_absolute_inertial(-93.0);
    longVolley(.11); //shoot last 3 diag
    turn_absolute_inertial(-90);
    inertial_drive(-43, 95);
    turn_absolute_inertial(-135);
    Expansion.set(true); //W
    SideExpansion.set(true);

    //*/

    /* OLD RUN
    turn_absolute_inertial(45.75);
    inertial_drive(29,90);
    
    // intake 3 stack
    FwVelocitySet(460, .95);
    spinIntake();
    inertial_drive(35.9,55); 
    turn_absolute_inertial(177.5);
    timeCtrl("driveb", .35);
    AutoRoller("red", 1);

    moveRot(.75, 60);
    turn_absolute_inertial(263);
    inertial_drive(37, 95);
    volley(450);
    turn_absolute_inertial(145);

    inertial_drive(19,85);
    turn_rel_inertial(1);
    turn_absolute_inertial(90.0);
    spinIntake();
    inertial_drive(22, 85);
    inertial_drive(30, 50);
    turn_absolute_inertial(270.5);
    stopIntake();
    timeCtrl("driveb", 1);
    AutoRoller("red", 2);

    // other side

    // intake first disc
    spinIntake();
    FwVelocitySet(440, .95);
    moveRot(.65,50); //.45
    turn_absolute_inertial(180.0);
    //timeCtrl("", .1);
    inertial_drive(40,90);
    //moveToPoint(132, 86 , 70);
    stopIntake();
    //turn_rel_inertial(4.5);
    volley(445); // shoot

    turn_absolute_inertial(-46.0);
    spinIntake();
    inertial_drive(24.5, 80);
    FwVelocitySet(445, .95);
    turn_absolute_inertial(-134.5);
    inertial_drive(37, 65);
    turn_absolute_inertial(120);
    volley(467); // shoot
    turn_absolute_inertial(220);

    spinIntake();
    inertial_drive(34, 80);
    inertial_drive(14, 65);
    FwVelocitySet(465, .95);
    turn_absolute_inertial(-270.0);
    inertial_drive(12, 50);
    stopIntake();
    volley(465); //shoot
    inertial_drive(-41, 95);
    turn_absolute_inertial(45);
    Expansion.set(true);
*/

    // volley
    /*
    FwVelocitySet(465, .95);
    turn_absolute_inertial(136);
    inertial_drive(12, 50);
    stopIntake();
    volley(465); //shoot

    turn_absolute_inertial(38);
    inertial_drive(-51, 95);
    turn_absolute_inertial(45);
    Expansion.set(true);
    timeCtrl("driveb", .3);
    */
    
  } else if(auto1 || HyperCarry.pressing()){
    // roller
    
    setFlywheelVel(574); 
    timeCtrl("driveb", .1);
    timeCtrl("intake", .4);
    moveRot(.53,50);

    turn_absolute_inertial(52.2);
    spinIntake();
    inertial_drive(27, 75);
    timeCtrl("", .1);
    turn_absolute_inertial(-23.6);
    //wait(500, msec);
    wait(200, msec);
    longVolley();

    // second volley
    setFlywheelVel(537);
    turn_absolute_inertial(49.2);
    spinIntake();
    inertial_drive(41, 70);
    turn_absolute_inertial(-47.2);
    wait(100, msec);
    longVolley();
    turn_absolute_inertial(43.5);
    setFlywheelVel(0);
    spinIntake();
    inertial_drive(64, 85);
    
    //2nd roller
    turn_absolute_inertial(-90);
    timeCtrl("driveb",.4);
    timeCtrl("intake", .4);
    
  } else if(auto2 || RollerSide.pressing()){
    // roller
    setFlywheelVel(587);
    timeCtrl("driveb", .1);
    timeCtrl("intake", .27);
    inertial_drive(10.5, 45);
    wait(150, msec);
    turn_absolute_inertial(-14.1);
    longVolley();

    setFlywheelVel(574); // 85
    ThreeStack.set(true);
    moveRot(.53,28);
    ThreeStack.set(false);
    spinIntake();
    timeCtrl("", 2);
    
    longVolley();

    moveRot(-1.05,40);
    setFlywheelVel(545); //570 35
    turn_absolute_inertial(49.0); 
    spinIntake();
    inertial_drive(15, 65);
    inertial_drive(24.5, 40);
    //Shoot 3
    turn_absolute_inertial(-36.1);
    longVolley();
    spinIntake();
    setFlywheelVel(530);
    wait(100, msec);
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
    setFlywheelVel(576);
    inertial_drive(-20, 60);
    turn_absolute_inertial(90);
    timeCtrl("driveb", .27);
    timeCtrl("intake", .41);
    moveRot(.5,60);
    turn_absolute_inertial(43);
    spinIntake();
    inertial_drive(27,70);

    // 1st volley
    turn_absolute_inertial(109.2);
    longVolley();
    turn_absolute_inertial(115);

    // intake 
    spinIntake();
    moveRot(1.4, 30);
    moveRot(-1.5, 30);
    turn_absolute_inertial(46.5);
    inertial_drive(38,55);
    inertial_drive(-19, 55);

    // 2nd volley
    setFlywheelVel(552);
    turn_absolute_inertial(119.3);
    longVolley();
    setFlywheelVel(0);
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
    turn_absolute_inertial(90);
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

  int defaultRPM =460; //470
  int angleRpm = 435;
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
      FwVelocitySet( flywheelRPM, .95 );
      setFlywheelVel(flywheelRPM);
    } else if(!toggle) {
      FwVelocitySet( 0, 0 );
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
      if (flywheelRPM == angleRpm){
        AngleAdjust.set(false);
        flywheelRPM = defaultRPM;
        FwVelocitySet(flywheelRPM, .95 );
        setFlywheelVel(flywheelRPM);
      }
      else{
        AngleAdjust.set(true);
        flywheelRPM = angleRpm;
        FwVelocitySet(flywheelRPM, .95 ); 
        setFlywheelVel(flywheelRPM);  
      }
    }

    if (r1_pressing) {
      spinIndex();
      Flywheel.spin(reverse,100,pct);
    }
    else if (r2_pressing) { 
      spinIntake();
      Flywheel.spin(fwd,100,pct);
    }
    else {
      stopIntake();
      stopFlywheel();
    }

    if (Controller1.ButtonUp.pressing()) {
      AngleAdjust.set(true);
      flywheelRPM=angleRpm;
      FwVelocitySet( flywheelRPM, .95 );
      setFlywheelVel(flywheelRPM);    
    }
    else if (Controller1.ButtonDown.pressing()) {
      AngleAdjust.set(false);
      flywheelRPM = defaultRPM;
      FwVelocitySet( flywheelRPM, .95 );
      setFlywheelVel(flywheelRPM);
      } 
    
    if(Controller1.ButtonLeft.pressing()){
      flywheelRPM-=50;
      FwVelocitySet( flywheelRPM, .95 );
      setFlywheelVel(flywheelRPM);
    } 
    else if (Controller1.ButtonRight.pressing()){
      flywheelRPM+=50;
      if(flywheelRPM>=600)
        flywheelRPM=600;
      FwVelocitySet( flywheelRPM, .95 );
      setFlywheelVel(flywheelRPM);
    }
    else {}

    if(Controller1.ButtonA.pressing()){
    } 
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
