#include "kalman.h"

Kalman1D::Kalman1D(
  double covariance,
  double noise_covariance,
  double initial_state,
  double initial_uncertainty
)
  : Q(covariance), R(noise_covariance)
{
  state = initial_state;
  P = initial_uncertainty;
}

void Kalman1D::update(double measurement) {
  // Find the difference between the filter's current state and the incoming state
  double error = measurement - state;

  // Calculate the extent to which we want to incorporate the incoming value
  // into our current state
  double kalman_gain = P * (1.0 / (P + R));

  // Incorporate the incoming value
  state += kalman_gain * error;

  // Update our uncertainty about our current state
  P = (1.0 - kalman_gain) * P + Q;
}

void Kalman1D::set_state(double value) {
  state = value;
  P = 0.0;
}