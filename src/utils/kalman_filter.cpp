
#include "utils/kalman_filter.hpp"

KalmanFilter::KalmanFilter() {}
void KalmanFilter::init(float initial_x, float initial_P, float process_noise,
                        float measurement_noise) {
  x = initial_x;
  P = P2 = init_P = initial_P;
  Q = process_noise;
  R = R2 = measurement_noise;
}

void KalmanFilter::print_state() {
  printf("  P:%f, Q:%f, R:%f\n", P, Q, R); //
}

__attribute__((section(".time_critical.ego_estimator")))
void KalmanFilter::predict(float u) {
  x = x + u * dt;
  P = P + Q;
  P2 = P2 + Q;
}

__attribute__((section(".time_critical.ego_estimator")))
void KalmanFilter::update(float z) {
  float K = P / (P + R);
  x = x + K * (z - x);
  P = (1 - K) * P;
}

__attribute__((section(".time_critical.ego_estimator")))
void KalmanFilter::update2(float z_gyro, float z_wheel) {
  float weight_gyro = P / (P + R);
  float weight_wheel = P2 / (P2 + R2);
  x = (weight_gyro * z_gyro + weight_wheel * z_wheel) /
      (weight_gyro + weight_wheel);
  P = (P * R2) / (R + R2);
  P2 = (P2 * R) / (R + R2);
}

__attribute__((section(".time_critical.ego_estimator")))
float KalmanFilter::get_state() {
  return x;
}

void KalmanFilter::reset(float reset_val) {
  x = reset_val;
  P = init_P;
}

void KalmanFilter::offset(float offset) {
  x = x + offset;
}