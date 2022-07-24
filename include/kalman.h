#ifndef __KALMAN_H
#define __KALMAN_H

// A simple single-variable Kalman filter.
class Kalman1D {
private:
  // Process nosie covariance
  const double Q;
  // Measurement noise covariance
  const double R;
  // State uncertainty
  double P;

public:
  // Our internal state
  double state;

  // Initializer for the class
  Kalman1D(double covariance, double noise_covariance, double initial_state, double initial_uncertainty);

  // Update the internal state of the filter given the new measurement value.
  void update(double measurement);

  // Set the state of the filter with complete certainty.
  void set_state(double value);
};

#endif