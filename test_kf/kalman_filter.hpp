#pragma once

#include <Eigen/Dense>

class KalmanFilter {
 public:
  KalmanFilter() { is_initialized_ = false; }

  ~KalmanFilter();

  void initKF(Eigen::VectorXd x) {
    x_ = x;
    is_initialized_ = true;
  }

  bool isInitialized() { return is_initialized_; }

  void setF(Eigen::MatrixXd F) { F_ = F; }
  void setU(Eigen::MatrixXd u) { u_ = u; }

  void setP(Eigen::MatrixXd P) { P_ = P; }

  void setQ(Eigen::MatrixXd Q) { Q_ = Q; }

  void predictionKF() {
    x_ = F_ * x_ + u_;
    P_ = F_ * P_ * F_.transpose() + Q_;
  }

  void setH(Eigen::MatrixXd H) { H_ = H; }

  void setR(Eigen::MatrixXd R) { R_ = R; }

  void measurementUpdateKF(const Eigen::VectorXd& z) {
    // z->measurement
    Eigen::VectorXd y = z - H_ * x_;
    Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
    // K -> kalman gain
    Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
    x_ = x_ + (K * y);
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_.size(), x_.size());
    P_ = (I - K * H_) * P_;
  }

  Eigen::VectorXd getX() { return x_; }

 private:
  bool is_initialized_;

  // state vector
  Eigen::VectorXd x_;
  // state transistion matrix
  Eigen::MatrixXd F_;
  Eigen::VectorXd u_;
  // state covariance matrix
  Eigen::MatrixXd P_;
  // process covariance matrix
  Eigen::MatrixXd Q_;
  // measurement matrix
  Eigen::MatrixXd H_;
  // measurement covariance matrix
  Eigen::MatrixXd R_;
  // // kalman gain
  // Eigen::MatrixXd K_;
};