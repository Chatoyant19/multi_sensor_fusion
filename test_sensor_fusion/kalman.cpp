#include "kalman.hpp"

Kalman::Kalman() { is_initialized_ = false; }

Kalman::~Kalman() {}

void Kalman::initlization(Eigen::VectorXd x) { x_ = x; }

bool Kalman::isInitialized() { return is_initialized_; }

void Kalman::setF(Eigen::MatrixXd F) { F_ = F; }

void Kalman::setU(Eigen::MatrixXd u) { u_ = u; }

void Kalman::setP(Eigen::MatrixXd P) { P_ = P; }

void Kalman::setQ(Eigen::MatrixXd Q) { Q_ = Q; }

void Kalman::prediction() {
  x_ = F_ * x_ + u_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void Kalman::setH(Eigen::MatrixXd H) { H_ = H; }

void Kalman::setR(Eigen::MatrixXd R) { R_ = R; }

void Kalman::measurementUpdateKF(const Eigen::VectorXd& z) {
  // z->measurement
  Eigen::VectorXd y = z - H_ * x_;
  Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
  // K -> kalman gain
  Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
  x_ = x_ + (K * y);
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K * H_) * P_;
}

void Kalman::measurementUpdateEKF(const Eigen::VectorXd& z) {
  Eigen::VectorXd h = Eigen::VectorXd(3);
  double rho = std::sqrt(x_(0) * x_(0) + x_(1) * x_(1));
  double theta = atan2(x_(1), x_(0));
  double rho_dot = (x_(0) * x_(2) + x_(1) * x_(3)) / rho;
  h << rho, theta, rho_dot;
  // z->measurement
  Eigen::VectorXd y = z - h;

  // compute H_
  calculateJacobin();

  Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
  // K -> kalman gain
  Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
  x_ = x_ + (K * y);
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K * H_) * P_;
}

void Kalman::calculateJacobin() {
  Eigen::MatrixXd H = Eigen::MatrixXd(3, 4);

  float x = x_(0);
  float y = x_(1);
  float v_x = x_(2);
  float v_y = x_(3);

  if (abs(x * x + y * y) < 0.0001) {
    H_ = H;
    return;
  }

  H << x / sqrt(x * x + y * y), y / sqrt(x * x + y * y), 0, 0,
      -y / (x * x + y * y), x / (x * x + y * y), 0, 0,
      (y * y * v_x - x * y * v_y) / pow((x * x + y * y), 3 / 2),
      (x * x * v_y - x * y * v_x) / pow((x * x + y * y), 3 / 2),
      x / sqrt((x * x + y * y)), y / sqrt((x * x + y * y));

  H_ = H;
}

Eigen::VectorXd Kalman::getX() { return x_; }