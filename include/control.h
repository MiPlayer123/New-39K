#ifndef __CONTROL_H
#define __CONTROL_H

#include "util.h"
#include "vex.h"
#include "util.h"
#include <string>

using namespace vex;

void spinIntake();

void longVolley();

void spinIndex();

void stopIntake();

void spinFlywheel();

void stopFlywheel();

void AutoRoller(std::string colour, int flag);

void timeCtrl(std::string control, float tim, float motorSpeed=100);

void volley(int speed, float timeIndex=1.8);

void FwVelocitySet( int velocityM, float predicted_drive );

int FwControlTask();

void setFlywheelVel(int rpm);

int flywheelControl();

int flyCtrl();

#endif