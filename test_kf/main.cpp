#include <iostream>

#include "kalman_filter.hpp"

bool getLidarData(double m_x, double m_y, double now_timestamp);

int main(int argc, char** argv) {
  double m_x = 0.0, m_y = 0.0;
  double now_timestamp = 0.0, last_timestamp = 0.0;
  KalmanFilter kf;
  while (getLidarData(m_x, m_y, now_timestamp)) {
    if (!kf.isInitialized()) {
      last_timestamp = now_timestamp;
      Eigen::VectorXd x(4, 1);
      x << m_x, m_y, 0.0, 0.0;
      kf.initKF(x);

      Eigen::VectorXd u(4, 1);
      u << 0.0, 0.0, 0.0, 0.0;
      kf.setU(u);

      Eigen::MatrixXd P(4, 4);
      P << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0,
          0.0, 0.0, 100.0;
      kf.setP(P);

      Eigen::MatrixXd Q(4, 4);
      Q << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
          0.0, 1.0;
      kf.setQ(Q);

      Eigen::MatrixXd H(2, 4);
      H << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0;
      kf.setH(H);

      // R is provided by sensor supplier
      Eigen::MatrixXd R(2, 2);
      R << 0.0225, 0.0, 0.0, 0.0225;
      kf.setR(R);
      continue;
    }

    double delta_t = now_timestamp - last_timestamp;
    last_timestamp = now_timestamp;
    Eigen::MatrixXd F(4, 4);
    F << 1.0, 0.0, delta_t, 0.0, 0.0, 1.0, 0.0, delta_t, 0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0;
    kf.setF(F);

    kf.predictionKF();

    Eigen::VectorXd z(2, 1);
    z << m_x, m_y;
    kf.measurementUpdateKF(z);

    // get result
    Eigen::VectorXd x_out = kf.getX();
    std::cout << "kalman out result: " << x_out(0) << " " << x_out(1)
              << std::endl;
  }
  return 0;
}

bool getLidarData(double m_x, double m_y, double now_timestamp) {}