using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor BaseLeftRear;
extern motor BaseLeftFront;
extern motor BaseRightRear;
extern motor BaseRightFront;
extern motor BaseLeftMid;
extern motor BaseRightMid;
extern limit Skills;
extern controller Controller1;
extern inertial Inertial;
extern rotation LTrack;
extern rotation RTrack;
extern motor Flywheel;
extern motor Intake;
extern digital_out Expansion;
extern digital_out AngleAdjust;
extern optical OpticalLeft;
extern rotation STrackO;
extern limit HyperCarry;
extern limit RollerSide;
extern limit FarSide;
extern optical OpticalRight;
extern digital_out SideExpansion;
extern rotation STrack;
extern digital_out ThreeStack;
extern inertial Inertial2;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );