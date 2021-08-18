#pragma once

#include "kalman.hpp"
#include "measurement_package.hpp"

class SensorFusion {
 public:
  SensorFusion();
  ~SensorFusion();

  void process(MeasurementPackage measurement_pack);
  Kalman kal_;

 private:
  bool is_initialized_;
  long last_timestamp_;
  Eigen::MatrixXd R_lidar_;
  Eigen::MatrixXd R_radar_;
  Eigen::MatrixXd L_lidar_;
  Eigen::MatrixXd H_lidar_;
};
