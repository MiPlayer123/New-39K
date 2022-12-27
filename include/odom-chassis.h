#include "vex.h"
#include "odometry.h"

extern double xTargetLocation;
extern double yTargetLocation;
extern double targetFacingAngle;

extern bool runChassisControl;
extern bool enableBreak;

void disableBreak();

void waitTilCompletion();

extern void driveTo(double xTarget, double yTarget, double targetAngle, double timeOutLength = 5000, double maxSpeed = 1.0);

extern void turnTo(double targetAngle, double timeOutLength = 2000);

extern void turnToPoint(double xCoordToFace, double yCoordToFace, double timeOutLength = 2000);

void setDrivePower(double theta);

void drivePID();
void turnPID();

int chassisControl();