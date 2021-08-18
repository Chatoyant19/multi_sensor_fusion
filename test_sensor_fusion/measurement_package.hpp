#pragma once

#include <Eigen/Dense>

class MeasurementPackage {
 public:
  long timestamp_;

  enum SensorType { LIDAR, RADAR } sensor_type_;

  Eigen::VectorXd measurement_values_;
};