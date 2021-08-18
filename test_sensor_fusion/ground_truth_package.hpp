#pragma once

#include <Eigen/Dense>

class GroundTruthPackage {
 public:
  enum SensorType { LIDAR, RADAR } sensor_type_;

  Eigen::VectorXd gt_values_;
};