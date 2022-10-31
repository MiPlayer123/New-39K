#include "control.h"
#include "chasis.h"

using namespace vex;

//Voids for auton 

void spinIntake(){
  Intake.spin(fwd, 80, pct);
}

void spinIndex(){
  Intake.spin(reverse, 80, pct);
}

void stopIntake(){
  Intake.stop(coast);
}

void spinFlywheel(){
  //Flywheel.spin(fwd, 11, volt);
  Flywheel.spin(fwd, 90,pct);
}

void stopFlywheel(){
  Flywheel.stop(coast);
}