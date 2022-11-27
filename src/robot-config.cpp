#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor BaseLeftRear = motor(PORT20, ratio6_1, true);
motor BaseLeftFront = motor(PORT1, ratio6_1, true);
motor BaseRightRear = motor(PORT19, ratio6_1, false);
motor BaseRightFront = motor(PORT6, ratio6_1, false);
motor BaseLeftMid = motor(PORT11, ratio6_1, true);
motor BaseRightMid = motor(PORT8, ratio6_1, false);
limit Skills = limit(Brain.ThreeWirePort.H);
controller Controller1 = controller(primary);
inertial Inertial = inertial(PORT13);
rotation LTrack = rotation(PORT18, true);
rotation RTrack = rotation(PORT17, false);
motor Flywheel = motor(PORT15, ratio6_1, true);
motor Intake = motor(PORT2, ratio6_1, true);
encoder STrack = encoder(Brain.ThreeWirePort.C);
digital_out Expansion = digital_out(Brain.ThreeWirePort.G);
digital_out AngleAdjust = digital_out(Brain.ThreeWirePort.F);
optical Optical = optical(PORT7);

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