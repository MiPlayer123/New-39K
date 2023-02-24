#include "control.h"
#include "chasis.h"

using namespace vex;

//Voids for auton 

void spinIntake(){
  Intake.spin(fwd, 100, pct);
}

void longVolley(float wait){
  stopIntake();
  timeCtrl("index", .28, 100); 
  timeCtrl("",wait);
  timeCtrl("index", .28, 100); 
  timeCtrl("",wait);
  timeCtrl("index", .3, 100);
  timeCtrl("", .1);
}

void spinIndex(){
  Intake.spin(reverse, 100, pct);
}

void stopIntake(){
  Intake.stop(coast);
}

void spinFlywheel(){
  //Flywheel.spin(fwd, 11, volt);
  Flywheel.spin(fwd, 60,pct);
}

void stopFlywheel(){
  Flywheel.stop(coast);
}

void AutoRoller(std::string colour, int flag){
  double timeout = .8;
  int count = 0;
  vex::color rollerColor;
  if(colour=="red")
    rollerColor = vex::red;
  else if(colour=="blue")
    rollerColor = vex::blue;
  else
    rollerColor = vex::white;
  if (flag == 1){
    while(OpticalLeft.color()!=rollerColor && count/1000 < timeout){
      spinIntake();
      swinging(10, 0,0);
      task::sleep(20);
      count+=20;
    }
    Intake.stop(brake);
  }
  else if (flag == 2) {
     while(OpticalRight.color()!=rollerColor && count/1000 < timeout){
      spinIntake();
      swinging(0, 10,0);
      task::sleep(20);
      count+=20;
    }
    Intake.stop(brake);
  }
}

void timeCtrl(std::string control, float tim, float motorSpeed){
  float mili = tim*1000;
  if(control == "drivef"){
    allBaseVoltage(true, 5);
    task::sleep(mili);
    allBaseVoltage(true, 0);
    brake_unchecked();
  }
  else if  (control == "driveb"){
    allBaseVoltage(false, 5);
    task::sleep(mili);
    allBaseVoltage(true, 0);
    brake_unchecked();
  }
  else if (control == "intake"){
    Intake.spin(fwd, motorSpeed, pct);
    task::sleep(mili);
    stopIntake();
  }
  else if (control == "index"){
    Intake.spin(reverse, motorSpeed, pct);
    task::sleep(mili);
    stopIntake();
  }
  else if (control == "shoot"){
    FwVelocitySet(470,.95);
    setFlywheelVel(470);
    task::sleep(mili);
    FwVelocitySet(0, 0);
  }
  else{
    task::sleep(mili);
  }
}

void volley(int speed, float timeIndex, int speedIndex){
  FwVelocitySet(speed, .95);
  setFlywheelVel(speed);
  while((Flywheel.velocity(rpm)-5)<speed){
    task::sleep(20);
  }
  timeCtrl("index", timeIndex, speedIndex);
  FwVelocitySet(0, 0);
  setFlywheelVel(0);
}

/* Flywheel TBH Controller */

// velocity measurement
float           motor_velocity;         ///< current velocity in rpm         
long            nSysTime_last;          ///< Time of last velocity calculation      

// TBH control algorithm variables
long            target_velocity;        ///< target_velocity velocity       
float           current_error;          ///< error between actual and target_velocity velocities
float           last_error;             ///< error last time update called          
float           gain;                   ///< gain
float           drive_out;                  ///< final drive out of TBH (0.0 to 1.0)
float           drive_at_zero;          ///< drive at last zero crossing
long            first_cross;            ///< flag indicating first zero crossing
float           drive_approx;           ///< estimated open loop drive

// final motor drive
long            motor_drive;            ///< final motor control value

/*Set the controller position */
void FwVelocitySet( int velocityM, float predicted_drive )
{
	// set target_velocity velocity (motor rpm)
	target_velocity = velocityM;

	// Set error so zero crossing is correctly detected
	current_error = target_velocity - motor_velocity;
	last_error    = current_error;

	// Set predicted open loop drive value
	drive_approx  = predicted_drive;
	// Set flag to detect first zero crossing
	first_cross   = 1;
	// clear tbh variable
	drive_at_zero = 0;
}

void FwCalculateSpeed()
{
  motor_velocity = Flywheel.velocity(rpm);
}

/*Update the velocity tbh controller variables*/
void FwControlUpdateVelocityTbh()
{
	// calculate error in velocity
	// target_velocity is desired velocity
	// current is measured velocity
	current_error = target_velocity - motor_velocity;

	// Calculate new control value
	drive_out =  drive_out + (current_error * gain);

	// Clip to the range 0 - 1.
	// We are only going forwards
	if( drive_out > 1 )
		drive_out = 1;
	if( drive_out < 0 )
		drive_out = 0;

	// Check for zero crossing
	if( sgn(current_error) != sgn(last_error) ) {
		// First zero crossing after a new set velocity command
		if( first_cross ) {
			// Set drive to the open loop approximation
			drive_out = drive_approx;
			first_cross = 0;
		}
		else
			drive_out = 0.5 * ( drive_out + drive_at_zero );

		// Save this drive value in the "tbh" variable
		drive_at_zero = drive_out;
	}
;
	// Save last error
	last_error = current_error;
}

int FwControlTask()
{
	// Set the gain
	gain = 0.0003; //0.00025

	while(1)
	{
		// Calculate velocity
		FwCalculateSpeed();

		// Do the velocity TBH calculations
		FwControlUpdateVelocityTbh() ;
		// Scale drive into the range the motors need
		motor_drive  = (drive_out * 12) + 0.0; //0.5

		// and finally set the motor con2trol value
		Flywheel.spin(fwd, motor_drive, volt);

		// Run at somewhere between 20 and 50mS
		task::sleep( 20 );
	}
  return 1;
}

int targetRPM;
bool flywheelRunning = false;

void setFlywheelVel(int rpm){
  if (rpm==0)
    flywheelRunning = false;
  else{
    flywheelRunning = true;
    targetRPM = rpm;
  }
}


int flywheelControl() {
  Flywheel.setBrake(coast);

  double kP = 0.007;//3;//14;
  double kD = 0.09;//2;//5;//.2;
  double kF = 1.05 / 300.0; //1.16
  double alpha; //.15
  double alpha2; //.1
  double trueRPM = 0;
  double filteredRPM = 0;
  double prevFilteredRPM = 0;
  double motorPower = 0;
  double prevMotorPower=0;
  //double slewLimit = 0.7; // volts / 10 msec. KEEP IN MIND UNITS!
  double error = 0;
  //double prevError = 0;
  double derivative = 0;
  //double prevD = 0;
  double prevDError = 0;
  double derror = 0;
  double prevFilteredRPM2=0;
  double filteredRPM2 = 0;
  // TUNE KP AND KD SO THAT THEY WORK FOR VOLTS!

  while (true) {

    if(targetRPM > 620 && filteredRPM<0.97*targetRPM){ //520
      kP = 0.004;//.007;
      kD = 0.1;//.09;
      kF = 1.0 / 300.0; //1.0
      alpha = .5; //.15
      alpha2 = .15; //.15
    }
    else {
      kP = 0.007;//3;//14;
      kD = 0.09;//2;//5;//.2;
      kF = 1.05 / 300.0; //1.16
      alpha = .7; //.15
      alpha2 = .2; //.1
    }

    double setRPM = targetRPM*6;
    
    // FLYWHEEL VELO CALCS
    trueRPM = Flywheel.velocity(rpm)*6;  

    filteredRPM = (alpha * trueRPM) + ((1 - alpha) * (prevFilteredRPM));

    prevFilteredRPM = filteredRPM;

    error = setRPM - filteredRPM;

    //derivative = error - prevError;

    filteredRPM2 = (alpha2 * trueRPM) + ((1 - alpha2)* (prevFilteredRPM2));

    prevFilteredRPM2 = filteredRPM2;
    
    derror = setRPM - filteredRPM2;

    derivative = derror - prevDError;

    derivative /= derivative >= 0 ? 1 : 4;
    
    //derivative = (alpha2 * derivative) + ((1 - alpha2) * (prevD));
    
    prevDError = derror;

    //prevError = error;

    motorPower = flywheelRunning ? error * kP + derivative * kD + setRPM *kF : 0;

    Brain.Screen.setCursor(8, 1);
    Brain.Screen.print("Tget: %.0f Actual: %.1f F1: %.1f F2: %.1f", setRPM, trueRPM, filteredRPM, filteredRPM2);
    Brain.Screen.setCursor(9, 1);
    Brain.Screen.print("kP: %.2f kD: %.2f kF: %.2f pwr: %.2f", error * kP, derivative * kD, setRPM * kF,  motorPower);

    Controller1.Screen.setCursor(3, 1);
    Controller1.Screen.print("Set: %.0f Actual: %.0lf", (float)targetRPM, Flywheel.velocity(rpm)); 

    if(motorPower <= 0) motorPower = 0;
    if(filteredRPM<0.88*targetRPM && flywheelRunning && motorPower<12)
      motorPower = 12.5;
      
      
    // SLEW RATE IMPLEMENTATION
    //double increment = motorPower - prevMotorPower;
    //if(std::abs(increment) > slewLimit) motorPower = prevMotorPower + (slewLimit * sgn(increment));
    
    prevMotorPower = motorPower;
    
    Flywheel.spin(forward, motorPower, volt);

    wait(10, msec);
  }
  return 1;
}

double II = .9; //there is slight oscilation, not much 1.1
double alpha = .2;
double estimatedpower_0 = 105.83;
double estimatedpower = 10;
double errorr;
double current_flywheel_speed;
double prevFilteredRPM = 0;
double dspeed;
double prevdspeed = 0;
double preverror;
double factor = 94.4881889764;
int flyCtrl(){
  while (true){
    if(flywheelRunning){
        //EMA filter
        current_flywheel_speed = (alpha * Flywheel.velocity(rpm)) + ((1 - alpha) * (prevFilteredRPM));
        prevFilteredRPM = current_flywheel_speed;
        if(current_flywheel_speed<0.95*targetRPM){
            Flywheel.spin(fwd, 12, volt);
        }
        else{
            //error
            errorr = targetRPM - current_flywheel_speed;


            //Integral factor
            //estimatedpower_0 += errorr*II;

            //derivative factor
            dspeed = (alpha * (errorr - preverror)) + ((1 - alpha) * (prevdspeed));
            prevdspeed = dspeed;

            estimatedpower_0 += dspeed*.5;

            //cap for power input
            if (estimatedpower_0 >127){
                estimatedpower_0 = 127;
            }
              
            if (std::signbit(preverror)!=std::signbit(errorr)){
                estimatedpower_0 = 10000/factor;
            }

            double scaled_power = estimatedpower_0/127;
            Flywheel.spin(fwd, 12*scaled_power, volt);

            preverror = errorr;
        }
    }
    else {
        Flywheel.spin(forward, 0, volt);
        errorr = 0;
        preverror = 0;
        estimatedpower = 10000/factor;
    }
    task::sleep( 10 );
  }
  return 1;
}