#pragma once

#include <Eigen/Dense>

class Kalman {
 public:
  Kalman();

  ~Kalman();

  void initlization(Eigen::VectorXd x);

  bool isInitialized();

  void setF(Eigen::MatrixXd F);
  void setU(Eigen::MatrixXd u);

  void setP(Eigen::MatrixXd P);

  void setQ(Eigen::MatrixXd Q);

  void prediction();

  void setH(Eigen::MatrixXd H);

  void setR(Eigen::MatrixXd R);

  void measurementUpdateKF(const Eigen::VectorXd& z);

  void measurementUpdateEKF(const Eigen::VectorXd& z);

  Eigen::VectorXd getX();

 private:
  void calculateJacobin();

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