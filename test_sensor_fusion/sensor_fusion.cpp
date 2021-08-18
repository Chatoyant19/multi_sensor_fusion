#include "sensor_fusion.hpp"

SensorFusion::SensorFusion() {
  is_initialized_ = false;
  last_timestamp_ = 0.0;

  H_lidar_ = Eigen::MatrixXd(2, 4);
  H_lidar_ << 1, 0, 0, 0, 0, 1, 0, 0;

  R_lidar_ = Eigen::MatrixXd(2, 2);
  R_lidar_ << 0.0225, 0.0, 0.0, 0.0225;

  R_radar_ = Eigen::MatrixXd(3, 3);
  R_radar_ << 0.09, 0.0, 0.0, 0.0, 0.0009, 0.0, 0., 0., 0.09;
}

SensorFusion::~SensorFusion() {}

void SensorFusion::process(MeasurementPackage measurement_pack) {
  if (is_initialized_) {
    // state vector
    Eigen::VectorXd x(4, 1);
    if (measurement_pack.sensor_type_ == MeasurementPackage::LIDAR) {
      x << measurement_pack.measurement_values_[0],
          measurement_pack.measurement_values_[1], 0, 0;
    } else if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      float rho = measurement_pack.measurement_values_[0];
      float phi = measurement_pack.measurement_values_[1];
      float rho_dot = measurement_pack.measurement_values_[2];
      float position_x = rho * cos(phi);

      // 避免运算时，0作为被除数
      if (position_x < 0.001 && position_x >= 0) {
        position_x = 0.001;
      } else if (position_x <= 0 && position_x >= -0.001) {
        position_x = -0.001;
      }
      float position_y = rho * sin(phi);
      if (position_y < 0.001 && position_y >= 0) {
        position_y = 0.001;
      } else if (position_y <= 0 && position_y >= -0.001) {
        position_y = -0.001;
      }
      float vx = rho_dot * cos(phi);
      float vy = rho_dot * sin(phi);
      x << position_x, position_y, vx, vy;
    }

    // initialized kalman filter
    kal_.initlization(x);

    Eigen::VectorXd u(4, 1);
    u << 0.0, 0.0, 0.0, 0.0;
    kal_.setU(u);

    Eigen::MatrixXd P(4, 4);
    P << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1000.0, 0.0, 0.0,
        0.0, 0.0, 1000.0;
    kal_.setP(P);

    Eigen::MatrixXd Q(4, 4);
    Q << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
        0.0, 1.0;
    kal_.setQ(Q);

    last_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }

  double delta_t =
      (measurement_pack.timestamp_ - last_timestamp_) / 1000000.0;  // nons -> s
  last_timestamp_ = measurement_pack.timestamp_;

  Eigen::MatrixXd F(4, 4);
  F << 1.0, 0.0, delta_t, 0.0, 0.0, 1.0, 0.0, delta_t, 0.0, 0.0, 1.0, 0.0, 0.0,
      0.0, 0.0, 1.0;
  kal_.setF(F);

  kal_.prediction();

  if (measurement_pack.sensor_type_ == MeasurementPackage::LIDAR) {
    kal_.setH(H_lidar_);
    kal_.setR(R_lidar_);
    kal_.measurementUpdateKF(measurement_pack.measurement_values_);
  } else if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    kal_.setR(R_radar_);
    kal_.measurementUpdateEKF(measurement_pack.measurement_values_);
  }
}