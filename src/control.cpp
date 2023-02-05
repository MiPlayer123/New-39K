#include "control.h"
#include "chasis.h"

using namespace vex;

//Voids for auton 

void spinIntake(){
  Intake.spin(fwd, 100, pct);
}

void longVolley(){
  stopIntake();
  timeCtrl("index", .29, 100); 
  timeCtrl("",.51);
  timeCtrl("index", .29, 100); 
  timeCtrl("",.51);
  timeCtrl("index", .29, 100);
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
  double timeout = 1.2;
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
      task::sleep(20);
      count+=20;
    }
    Intake.stop(brake);
  }
  else if (flag == 2) {
     while(OpticalRight.color()!=rollerColor && count/1000 < timeout){
      spinIntake();
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
    FwVelocitySet(450,.95);
    task::sleep(mili);
    FwVelocitySet(0, 0);
  }
  else{
    task::sleep(mili);
  }
}

void volley(int speed, float timeIndex){
  FwVelocitySet(speed, .95);
  while((Flywheel.velocity(rpm)-5)<speed){
    task::sleep(20);
  }
  timeCtrl("index", timeIndex);
  FwVelocitySet(0, 0);
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

int flywheelControl() {
  Flywheel.setBrake(coast);

  int targetRPM = 480; // initial RPM for flywheel

  bool flywheelRunning = true;

  double alpha = 0.7;
  double kP = 5;
  double kD = .8;
  double kF = 1.0 / 600.0;
  double trueRPM = 0;
  double filteredRPM = 0;
  double prevFilteredRPM = 0;
  double motorPower = 0;
  //double prevMotorPower = 0;
  //double slewLimit = 0.1; // volts / 10 msec. KEEP IN MIND UNITS!
  double error = 0;
  double prevError = 0;
  double derivative = 0;
  // TUNE KP AND KD SO THAT THEY WORK FOR VOLTS!

  while (true) {
    
    // FLYWHEEL VELO CALCS
    trueRPM = Flywheel.velocity(rpm);  

    filteredRPM = (alpha * trueRPM) + ((1 - alpha) * (prevFilteredRPM));

    error = targetRPM - filteredRPM;

    derivative = error - prevError;

    derivative /= derivative >= 0 ? 1 : 4;

    prevError = error;

    motorPower = flywheelRunning ? error * kP + derivative * kD + targetRPM * kF : 0;
    
    // SLEW RATE IMPLEMENTATION
    //double increment = motorPower - prevMotorPower;
    /*
    if (fabs(increment) > slewLimit) {
      motorPower = prevMotorPower + slewLimit * sgn(increment);
    }*/
    
    Flywheel.spin(forward, motorPower, volt);

    wait(10, msec);
  }
  return 1;
}