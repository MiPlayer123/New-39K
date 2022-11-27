#include "chasis.h"

//For PID turns
#define TURN_KP 0.03
#define TURN_KI 0.0015//25  //0.0018
#define TURN_KD 0.0001 //0.001
#define TURN_MAX_A (BASE_MAX_V / 0.1)
#define TURN_MAX_V (BASE_MAX_V * 0.7)
#define TURN_MIN_V 3

//For main inertial_drive
#define   kp 6.5 //8 
#define   ki .007 //.5
#define   kd 0.2 //.45
#define integral_threshold 10
#define kp_c .43 //.45

mutex heading_mtx;

// Filter to track our rotation
Kalman1D heading_filter(1.0, 0.50, 0.0, 0.0);

void initialize() {
// Start a daemon to update the filter in the background
  thread([]() {
    // Set our initial rotation to 0 regardless of the position of the robot
    Inertial.setRotation(0, degrees);
    
    while(true) {
      // Obtain our current rotation according to the sensor
      double measured_rotation = Inertial.rotation();

      // Update the filter
      heading_mtx.lock();
      heading_filter.update(measured_rotation);
      heading_mtx.unlock();

      // Wait 5ms before the next update
      wait(5, msec);
    }
  })
  // Allow the process to run in the background
  .detach();
  
  /*
  thread([]() {
     xPos = GPS.xPosition();
     yPos = GPS.yPosition();
  }).detach();
  */
}

// Get the current rotation of the robot by querying the state of the filter
double get_rotation() {
  heading_mtx.lock();
  double rotation = heading_filter.state;
  heading_mtx.unlock();
  return rotation;
}

void brake_unchecked() {
  BaseLeftRear.stop(brakeType::brake);
  BaseLeftFront.stop(brakeType::brake);
  BaseRightRear.stop(brakeType::brake);
  BaseRightFront.stop(brakeType::brake);
}

double getDist(){
  double leftAvg = (BaseLeftFront.position(turns) + BaseLeftMid.position(turns) + BaseLeftRear.position(turns))/3;
  double rightAvg = (BaseRightFront.position(turns) + BaseRightMid.position(turns) + BaseRightRear.position(turns))/3;
  double avg = (leftAvg+rightAvg)/2;
  double dist = avg * (3.25) * M_PI * (3.0/5.0);
  return dist;
}

// Turn to an absolute rotation
void turn_absolute_inertial(double target) {
  double last_error = 1000;
  double last_output = 0;
  double integral = 0;

  //long ticks = 0;
  long brake_cycles = 0;

  while (true) {
    // Calculate the error
    double error = target - get_rotation();

    // Initialize the last error if it was not already
    if (ae(last_error, 1000)) {
      last_error = error;
    }

    // Calculate the integral and derivative if we're within the bound
    double derivative;
    if (fabs(error) > 10) {
      integral = 0;
      derivative = 0;
    } else {
      integral += error * BASE_DT;
      derivative = (error - last_error) / BASE_DT;
    }
    
    // Calculate the raw output and update the last erro
    double raw_output;
    
    raw_output = TURNING_RADIUS * (TURN_KP * error + TURN_KI * integral + TURN_KD * derivative); // in/s
    
    last_error = error;

    // Constrain acceleration between iterations and calculate the output
    double acceleration = clamp((raw_output - last_output) / BASE_DT, -TURN_MAX_A, TURN_MAX_A); // in/s/s
    double output = last_output + acceleration * BASE_DT;

    // Constrain the maximum velocity
    // Maximum velocity we can have in order to decelerate to min_veloc with max_accel
    double vmax = sqrt(sq(BASE_MIN_V) + 2 * TURN_MAX_A * fabs(error));

    // Constrain this theoretical maximum with the given bounds
    vmax = clamp(vmax, BASE_MIN_V + EPSILON, 100);
    last_output = (output = clamp(output, -vmax, vmax));

    // Constrain the minimum velocity
    output = ithreshold(output, TURN_MIN_V);
    output = clamp(output / MOTOR_PERCENT_TO_IN_PER_SEC, -80, 80);

    // 
    if (fabs(error) < 1) {
      brake_unchecked();
      brake_cycles += 1;
    } else {
      spin(&BaseLeftRear, output);
      spin(&BaseLeftFront, output);
      spin(&BaseLeftMid, output);
      spin(&BaseRightRear, -output);
      spin(&BaseRightFront, -output);
      spin(&BaseRightMid, -output);
      brake_cycles = 0;
    }
    if (brake_cycles > 10 && fabs(error) < 1) {
      break;
    }
    wait(BASE_DT, sec);
  }
  BaseLeftFront.stop(vex::brakeType::brake);
  BaseRightFront.stop(vex::brakeType::brake);
  BaseRightRear.stop(vex::brakeType::brake);
  BaseLeftRear.stop(vex::brakeType::brake);
  BaseLeftMid.stop(brake);
  BaseRightMid.stop(brake);
}

void turn_rel_inertial(double target) {
  turn_absolute_inertial(get_rotation() + target);
}

void rotationDrive(double dist, double speed){
  double startPosR = 4*(BaseLeftFront.position(rev) + BaseLeftMid.position(rev) + BaseLeftRear.position(rev))/3; 
  double changePosR = 0; 
  double startPosL = 4*(BaseRightFront.position(rev) + BaseRightMid.position(rev) + BaseRightRear.position(rev))/3;
  double changePosL = 0; 
  BaseLeftFront.spin(fwd,12*speed/100, volt);
  BaseLeftMid.spin(fwd,12*speed/100, volt);
  BaseLeftRear.spin(fwd,12*speed/100, volt);
  BaseRightFront.spin(fwd,12*speed/100, volt);
  BaseRightMid.spin(fwd,12*speed/100, volt);
  BaseRightRear.spin(fwd,12*speed/100, volt);

  while((changePosL + changePosR)/2 <dist){
    changePosR = (4*(BaseLeftFront.position(rev) + BaseLeftMid.position(rev) + BaseLeftRear.position(rev))/3)-startPosR; 
    changePosL = (4*(BaseRightFront.position(rev) + BaseRightMid.position(rev) + BaseRightRear.position(rev))/3)-startPosL; 
    wait(5,msec);
  }
  brake_unchecked(); 
}

void moveRot (float rot, float speed)
{
  BaseLeftRear.rotateFor(rot, rotationUnits::rev, speed, velocityUnits::pct, false);
  BaseLeftFront.rotateFor(rot, rotationUnits::rev, speed, velocityUnits::pct, false);
  BaseRightFront.rotateFor(rot, rotationUnits::rev, speed, velocityUnits::pct, false);
  BaseRightMid.rotateFor(rot, rotationUnits::rev, speed, velocityUnits::pct, false);
  BaseLeftMid.rotateFor(rot, rotationUnits::rev, speed, velocityUnits::pct, false);
  BaseRightRear.rotateFor(rot, rotationUnits::rev, speed, velocityUnits::pct, true);
}

void turnRot (float rot, float speed){
  BaseLeftRear.rotateFor(rot, rotationUnits::rev, speed, velocityUnits::pct, false);
  BaseLeftFront.rotateFor(rot, rotationUnits::rev, speed, velocityUnits::pct, false);
  BaseRightRear.rotateFor(-rot, rotationUnits::rev, speed, velocityUnits::pct, false);
  BaseRightFront.rotateFor(-rot, rotationUnits::rev, speed, velocityUnits::pct, true);
  
  BaseLeftFront.stop(vex::brakeType::brake);
  BaseRightFront.stop(vex::brakeType::brake);
  BaseRightRear.stop(vex::brakeType::brake);
  BaseLeftRear.stop(vex::brakeType::brake);
}

void inertial_drive(double target, double speed) {
  BaseRightRear.setPosition(0, turns); 
  BaseLeftRear.setPosition(0, turns);
  BaseRightFront.setPosition(0, turns);
  BaseLeftFront.setPosition(0, turns);
  BaseLeftMid.setPosition(0, turns);
  BaseRightMid.setPosition(0, turns);
  LTrack.setPosition(0, turns); 
  RTrack.setPosition(0, turns);
  //Starting pos
  double angle = get_rotation();

  // Accumulated error
  double integral_c = 0;
  double last_error;
  double derivative;
  double integral;

  while (true) {
    // Calculate the error
    double error_c = angle - get_rotation();
    double error1 = target - getDist(); //LTrack.position(turns) * 2.8 * M_PI
    double error2 = target - getDist(); //RTrack.position(turns) * 2.8 * M_PI
    double error;
  
    error = (error1 + error2) / 2;

    // Get the turn output
    double raw_output_correct = (TURN_KP * error_c + 0.1 * integral_c); // in/s
    integral_c += error_c * BASE_DT;

    if (fabs(error) > integral_threshold) {
      integral = 0;
      derivative = 0;
    } else {
      integral += error * BASE_DT;
      derivative = (error - last_error) / BASE_DT;
    }
    
    double raw_output = kp * error + ki * integral + kd * derivative; // in/s
    last_error = error;

    // This is the extent to which we want to turn; a low value means no turns
    double factor = 0.1 + fabs(error_c) / 45;
    raw_output_correct *= factor; //old error
    raw_output_correct = error_c*kp_c; //new ange error
    //double correct_output = 2 * clamp(raw_output_correct, -speed, speed);

    if(raw_output > speed) raw_output = speed; //Limit speed
     else if(raw_output < -speed) raw_output = -speed; 
    
     //Normal speed
     if(target!=0){ //fwd correct
      BaseLeftFront.spin(vex::directionType::fwd, raw_output + raw_output_correct, vex::velocityUnits::pct);
      BaseLeftRear.spin(vex::directionType::fwd, raw_output + raw_output_correct, vex::velocityUnits::pct);
      BaseLeftMid.spin(vex::directionType::fwd, raw_output + raw_output_correct, vex::velocityUnits::pct);
      BaseRightFront.spin(vex::directionType::fwd, raw_output - raw_output_correct, vex::velocityUnits::pct);
      BaseRightRear.spin(vex::directionType::fwd, raw_output - raw_output_correct, vex::velocityUnits::pct);
      BaseRightMid.spin(vex::directionType::fwd, raw_output - raw_output_correct, vex::velocityUnits::pct);
     }
     else{ //rev correct
      BaseLeftFront.spin(vex::directionType::fwd, raw_output + raw_output_correct*0, vex::velocityUnits::pct);
      BaseLeftRear.spin(vex::directionType::fwd, raw_output + raw_output_correct*0, vex::velocityUnits::pct);
      BaseLeftMid.spin(vex::directionType::fwd, raw_output + raw_output_correct*0, vex::velocityUnits::pct);
      BaseRightFront.spin(vex::directionType::fwd, raw_output - raw_output_correct*0, vex::velocityUnits::pct);
      BaseRightRear.spin(vex::directionType::fwd, raw_output - raw_output_correct*0, vex::velocityUnits::pct);
      BaseRightMid.spin(vex::directionType::fwd, raw_output - raw_output_correct*0, vex::velocityUnits::pct);
     }

		if(std::abs(error) <= .5){
      if (speed==99){
        BaseLeftFront.stop(vex::brakeType::coast);
        BaseRightFront.stop(vex::brakeType::coast);
        BaseRightRear.stop(vex::brakeType::coast);
        BaseLeftRear.stop(vex::brakeType::coast);
        BaseRightMid.stop(vex::brakeType::coast);
        BaseLeftMid.stop(vex::brakeType::coast);
        break;
      }
      else{
        BaseLeftFront.stop(vex::brakeType::brake);
        BaseRightFront.stop(vex::brakeType::brake);
        BaseRightRear.stop(vex::brakeType::brake);
        BaseLeftRear.stop(vex::brakeType::brake);
        BaseLeftMid.stop(brake);
        BaseRightMid.stop(brake);
        break;
      }
		}
  }
}

void allBaseVoltage(bool dirFwd, double v){
  if(dirFwd){ 
    BaseLeftFront.spin(fwd, v, volt); 
    BaseLeftRear.spin(fwd, v, volt);
    BaseRightFront.spin(fwd, v, volt); 
    BaseRightRear.spin(fwd, v, volt);
    BaseRightMid.spin(fwd, v, volt); 
    BaseLeftMid.spin(fwd, v, volt);
  }
  else {
    BaseLeftFront.spin(reverse, v, volt); 
    BaseLeftRear.spin(reverse, v, volt);
    BaseRightFront.spin(reverse, v, volt); 
    BaseRightRear.spin(reverse, v, volt);
    BaseRightMid.spin(reverse, v, volt); 
    BaseLeftMid.spin(reverse, v, volt);
  }
}

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