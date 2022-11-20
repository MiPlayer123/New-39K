#ifndef __UTIL_H
#define __UTIL_H

#include <cmath>
#include <ctime>
#include "vex.h"

extern const double EPSILON;
extern const double TRACKING_POINT_TO_ODO_LEFT; // in
extern const double TRACKING_POINT_TO_ODO_RIGHT; // in
extern const double TRACKING_POINT_TO_ODO_DRIFT; // in
extern const double TURNS_TO_INCHES; // in/turn
extern const double MOVEMENT_THRESHOLD; // in
extern const double THETA_THRESHOLD; // radians
extern const double M_2PI;
extern const double TURNING_RADIUS; // in
extern const double LIGHT_SENSOR_GAP; // in
extern const double RPM_TO_INCHES_PER_SEC;
extern const double MOTOR_PERCENT_TO_IN_PER_SEC;

// Power the base with the given left and right percentages
void drive(double left, double right, vex::brakeType brake_type=brake);

// Spin the given motor with the given power (in units of percent). If the power
// is very close to 0 then the brake is applied.
void spin(motor *mot, double veloc, vex::brakeType brake_type=brake);

// Whether or not we're running skills.
bool is_skills();

// Returns true if a and b are within epsilon of each other as defined in consts.h.
bool ae(double a, double b);

// Clamps x between the given minimum and maximum.
double clamp(double x, double mn, double mx);

double iclamp(double x, double lim);

// Performs an inverse threshold, meaning that if |x| < threshold then threshold is
// returned.
double ithreshold(double x, double threshold);

// Clamps an angle (in radians) between -pi and pi.
double clamp_angle(double theta);

// Returns the current system time in seconds.
long double cts();

// Returns the sign of x: 1 if positive, -1 if negative, or 0 if x == 0.
int sign(double x);

// Returns the square of x.
double sq(double x);

// If |x| < threshold, then 0 is returned, else x is returned.
double threshold(double x, double threshold);

int sgn(double v);

#endif