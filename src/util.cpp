#include "util.h"
#include <cmath>

const double EPSILON = 1e-5;
const double TRACKING_POINT_TO_ODO_LEFT = 7.48;
const double TRACKING_POINT_TO_ODO_RIGHT = 7.48;
const double TRACKING_POINT_TO_ODO_DRIFT = 5.12;
const double TURNS_TO_INCHES = 3.25*M_PI*(3.0/5.0);//M_PI*4;
const double MOVEMENT_THRESHOLD = 0.05;
const double THETA_THRESHOLD = 0.01;
const double M_2PI = 2.0 * M_PI;
const double TURNING_RADIUS = 5.875; // old: 7.90
const double LIGHT_SENSOR_GAP = 5.551;
const double RPM_TO_INCHES_PER_SEC = TURNS_TO_INCHES / 60.0;
const double MOTOR_PERCENT_TO_IN_PER_SEC = 0.436;

void drive(double left, double right, vex::brakeType brake_type) {
  spin(&BaseLeftRear, left, brake_type);
  spin(&BaseLeftFront, left, brake_type);
  spin(&BaseRightRear, right, brake_type);
  spin(&BaseRightFront, right, brake_type);
}

void spin(motor *mot, double veloc, vex::brakeType brake_type) {
  if (ae(veloc, 0)) {
    mot->stop(brake_type);
  } else if (veloc < 0) {
    mot->spin(directionType::rev, fabs(veloc), percent);
  } else {
    mot->spin(directionType::fwd, veloc, percent);
  }
}

bool is_skills() {
  return Skills.pressing();
}

bool ae(double a, double b) {
  return fabs(a - b) < EPSILON;
}

double clamp(double x, double mn, double mx) {
  if (x > mx) {
    return mx;
  } else if (x < mn) {
    return mn;
  }

  return x;
}

double iclamp(double x, double lim) {
  if (x > -lim && x < 0) {
    return -lim;
  } else if (x < lim && x > 0) {
    return lim;
  }
  return x;
}

double ithreshold(double x, double threshold) {
  if (x < 0 && x > -threshold) {
    return -threshold;
  } else if (x > 0 && x < threshold) {
    return threshold;
  } else {
    return x;
  }
}

double clamp_angle(double theta) {
  theta = fmodf(theta, M_2PI);

  if (theta < -M_PI) {
    return theta + M_2PI;
  } else if (theta > M_PI) {
    return theta - M_2PI;
  }

  return theta;
}

long double cts() {
  long double current_time = (long double)clock();
  return current_time / 10.0;
}

int sign(double x) {
  return x > 0 ? 1 : (x < 0 ? -1 : 0);
}

double sq(double x) {
  return x * x;
}

double threshold(double x, double threshold) {
  if (fabs(x) < threshold) {
    return 0;
  } else {
    return x;
  }
}

int sgn(double v) {
  if (v < 0) return -1;
  if (v > 0) return 1;
  return 0;
}