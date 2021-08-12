#include <iostream>

#include "extended_kalman_filter.hpp"

bool getRadarData(double rho, double theta, double rho_dot,
                  double now_timestamp);

int main(int argc, char** argv) {
  double rho = 0.0, theta = 0.0, rho_dot = 0.0;
  double now_timestamp = 0.0, last_timestamp = 0.0;
  // KalmanFilter kf;
  ExtendedKalmanFilter ekf;
  while (getRadarData(rho, theta, rho_dot, now_timestamp)) {
    if (!ekf.isInitialized()) {
      last_timestamp = now_timestamp;
      Eigen::VectorXd x(4, 1);
      double m_x = rho * cos(theta);
      double m_y = rho * sin(theta);
      double m_v_x = rho_dot * cos(theta);
      double m_v_y = rho_dot * sin(theta);
      x << m_x, m_y, m_v_x, m_v_y;
      ekf.initEKF(x);

      Eigen::VectorXd u(4, 1);
      u << 0.0, 0.0, 0.0, 0.0;
      ekf.setU(u);

      Eigen::MatrixXd P(4, 4);
      P << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0,
          0.0, 0.0, 10.0;
      ekf.setP(P);

      Eigen::MatrixXd Q(4, 4);
      Q << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
          0.0, 1.0;
      ekf.setQ(Q);

      // Eigen::MatrixXd H(2, 4);
      // H << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      // 0.0,
      //     0.0, 0.0;
      // kf.setH(H);

      // R is provided by sensor supplier
      Eigen::MatrixXd R(3, 3);
      R << 0.09, 0.0, 0.0, 0.0, 0.0009, 0.0, 0., 0., 0.09;
      ekf.setR(R);
      continue;
    }

    double delta_t = now_timestamp - last_timestamp;
    last_timestamp = now_timestamp;
    Eigen::MatrixXd F(4, 4);
    F << 1.0, 0.0, delta_t, 0.0, 0.0, 1.0, 0.0, delta_t, 0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0;
    ekf.setF(F);

    ekf.predictionEKF();

    Eigen::VectorXd z(4, 1);
    z << rho, theta, rho_dot;
    ekf.measurementUpdateEKF(z);

    // get result
    Eigen::VectorXd x_out = ekf.getX();
    std::cout << "extended kalman out result: " << x_out(0) << " " << x_out(1)
              << std::endl;
  }
  return 0;
}

bool getRadarData(double rho, double theta, double rho_dot,
                  double now_timestamp) {}