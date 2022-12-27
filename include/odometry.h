#include "vex.h"
#include "chasis.h"

//The current angle of the bot (RADIANS)
extern double currentAbsoluteOrientation;

//The global position of the bot (INCHES)
extern double xPosGlobal;
extern double yPosGlobal;

//Gloval vars for dynamic starting position 
extern double THETA_START;

extern double X_START;
extern double Y_START;

//The odometry function
int positionTracking();