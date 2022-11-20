#ifndef __CONTROL_H
#define __CONTROL_H

#include "util.h"
#include "vex.h"
#include "util.h"
#include <string>

using namespace vex;

void spinIntake();

void spinIndex();

void stopIntake();

void spinFlywheel();

void stopFlywheel();

void AutoRoller(std::string colour);

void FwVelocitySet( int velocityM, float predicted_drive );

task FwControlTask();

#endif