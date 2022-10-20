#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor BaseLeftRear = motor(PORT1, ratio18_1, false);
motor BaseLeftFront = motor(PORT6, ratio18_1, false);
motor BaseRightRear = motor(PORT8, ratio18_1, true);
motor BaseRightFront = motor(PORT7, ratio18_1, true);
motor Flywheel1 = motor(PORT11, ratio6_1, true);
motor Intake = motor(PORT4, ratio6_1, true);
limit Skills = limit(Brain.ThreeWirePort.H);
rotation STrackO = rotation(PORT16, false);
controller Controller1 = controller(primary);
inertial Inertial = inertial(PORT5);
rotation LTrackO = rotation(PORT18, true);
rotation RTrackO = rotation(PORT17, false);
motor Flywheel2 = motor(PORT15, ratio6_1, false);
motor Intake2 = motor(PORT19, ratio6_1, false);
encoder STrack = encoder(Brain.ThreeWirePort.A);
encoder RTrack = encoder(Brain.ThreeWirePort.C);
encoder LTrack = encoder(Brain.ThreeWirePort.E);
digital_out Expansion = digital_out(Brain.ThreeWirePort.G);

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