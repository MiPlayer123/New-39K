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
digital_out Expansion = digital_out(Brain.ThreeWirePort.G);
digital_out AngleAdjust = digital_out(Brain.ThreeWirePort.D);
optical OpticalLeft = optical(PORT7);
rotation STrackO = rotation(PORT3, false);
limit HyperCarry = limit(Brain.ThreeWirePort.A);
limit RollerSide = limit(Brain.ThreeWirePort.B);
limit FarSide = limit(Brain.ThreeWirePort.C);
optical OpticalRight = optical(PORT5);
digital_out SideExpansion = digital_out(Brain.ThreeWirePort.E);
rotation STrack = rotation(PORT4, false);
digital_out ThreeStack = digital_out(Brain.ThreeWirePort.F);
inertial Inertial2 = inertial(PORT16);

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