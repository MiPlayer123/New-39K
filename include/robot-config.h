using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor BaseLeftRear;
extern motor BaseLeftFront;
extern motor BaseRightRear;
extern motor BaseRightFront;
extern motor Flywheel1;
extern motor Intake;
extern limit Skills;
extern digital_out Index;
extern rotation STrack;
extern controller Controller1;
extern inertial Inertial;
extern rotation LTrack;
extern rotation RTrack;
extern motor Flywheel2;
extern motor Turret;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );