#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor BaseLeftRear = motor(PORT14, ratio6_1, true);
motor BaseLeftFront = motor(PORT13, ratio6_1, true);
motor BaseRightRear = motor(PORT20, ratio6_1, false);
motor BaseRightFront = motor(PORT2, ratio6_1, false);
motor Flywheel1 = motor(PORT1, ratio6_1, false);
motor Intake = motor(PORT18, ratio6_1, false);
limit Skills = limit(Brain.ThreeWirePort.H);
digital_out Index = digital_out(Brain.ThreeWirePort.A);
rotation STrack = rotation(PORT7, false);
controller Controller1 = controller(primary);
inertial Inertial = inertial(PORT19);
rotation LTrack = rotation(PORT9, true);
rotation RTrack = rotation(PORT16, false);
motor Flywheel2 = motor(PORT12, ratio6_1, true);
motor Turret = motor(PORT10, ratio36_1, false);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}