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
extern rotation STrackO;
extern controller Controller1;
extern inertial Inertial;
extern rotation LTrackO;
extern rotation RTrackO;
extern motor Flywheel;
extern motor Intake;
extern encoder STrack;
extern encoder RTrack;
extern encoder LTrack;
extern digital_out Expansion;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );