#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor BaseLeftRear = motor(PORT2, ratio6_1, true);
motor BaseLeftFront = motor(PORT6, ratio6_1, true);
motor BaseRightRear = motor(PORT3, ratio6_1, false);
motor BaseRightFront = motor(PORT7, ratio6_1, false);
motor BaseLeftMid = motor(PORT11, ratio6_1, true);
motor BaseRightMid = motor(PORT8, ratio6_1, false);
limit Skills = limit(Brain.ThreeWirePort.H);
rotation STrackO = rotation(PORT16, false);
controller Controller1 = controller(primary);
inertial Inertial = inertial(PORT5);
rotation LTrackO = rotation(PORT18, true);
rotation RTrackO = rotation(PORT17, false);
motor Flywheel = motor(PORT15, ratio6_1, true);
motor Intake = motor(PORT1, ratio6_1, false);
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