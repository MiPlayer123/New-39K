#ifndef __BOT_H
#define __BOT_H

#include "kalman.h"
#include "control.h"
#include <string>

#define BASE_DT 0.005
#define BASE_INTEGRAL_THRESHOLD 20

#define BASE_MIN_V 3 // in/s
#define BASE_MAX_V 45 // in/s
#define BASE_MAX_A (BASE_MAX_V / 0.1) // in/s/s

// Initialize the chasis. Must be called once and only once at the start of the program.
void initialize();

double get_rotation();

void brake_unchecked();

double getDist();

// Turns the robot to an absolute rotation.
void turn_absolute_inertial(double target);

// Turns the robot a relative number of degrees.
void turn_rel_inertial(double target);

//PID drive with inertial correction
void inertial_drive(double target, double speed);

//BEN ROTATION
void rotationDrive(double dist, double speed);

//Move given rotations
void moveRot (float rot, float speed);

//Turn Rotations
void turnRot (float rot, float speed);

//Base voltage   
void allBaseVoltage(bool dirFwd, double v);

//Swing turn
void swinging(double leftPower, double rightPower, double gogoAngle);

#endif