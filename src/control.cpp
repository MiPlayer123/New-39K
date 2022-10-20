#include "control.h"
#include "chasis.h"

using namespace vex;

//Voids for auton 

void spinIntake(){
  Intake.spin(fwd, 100, pct);
}

void stopIntake(){
  Intake.stop(coast);
}

void spinFlywheel(){
  Flywheel1.spin(fwd, 12, volt);
  Flywheel2.spin(fwd, 12, volt);
}

void stopFlywheel(){
  Flywheel1.stop(coast);
  Flywheel2.stop(coast);
}