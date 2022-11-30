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

void timeCtrl(std::string control, float tim);

void volley(int speed, float timeIndex=2);

void FwVelocitySet( int velocityM, float predicted_drive );

int FwControlTask();

#endif