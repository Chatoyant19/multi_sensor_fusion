// lidar and radar
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "ground_truth_package.hpp"
#include "measurement_package.hpp"
#include "sensor_fusion.hpp"

int main(int argc, char** argv) {
  std::string input_file_name =
      "/home/wd/project/multi_seneor_fusion/test_sensor_fusion/"
      "sample-laser-radar-measurement-data-2.txt";

  std::ifstream input_file(input_file_name.c_str(), std::ifstream::in);
  if (!input_file.is_open()) {
    std::cout << "failed to open file" << std::endl;
    return -1;
  }

  std::vector<GroundTruthPackage> groundtruth_package_list;
  std::vector<MeasurementPackage> measurement_package_list;

  std::string line;
  while (std::getline(input_file, line)) {
    std::istringstream iss(line);

    GroundTruthPackage gt_package;
    MeasurementPackage measurement_package;

    std::string sensor_type;

    iss >> sensor_type;
    // std::cout << "sensor_type: " << sensor_type << " ";
    if (sensor_type.compare("L") == 0) {
      // 2nd element is x, 3rd element is y, 4th element is timestamp
      float m_x, m_y;
      long long timestamp_l;

      measurement_package.sensor_type_ = MeasurementPackage::LIDAR;

      iss >> m_x;
      iss >> m_y;
      measurement_package.measurement_values_ = Eigen::VectorXd(2);
      measurement_package.measurement_values_ << m_x, m_y;

      iss >> timestamp_l;
      measurement_package.timestamp_ = timestamp_l;

      // std::cout << m_x << " " << m_y << " " << timestamp_l << " ";

      // measurement_package_list.emplace_back(measurement_package);
    } else if (sensor_type.compare("R") == 0) {
      // 2nd element is pho, 3rd element is phi, 4th element is pho_dot, 5th
      // element is timestamp
      float pho, phi, pho_dot;
      long long timestamp_r;

      measurement_package.sensor_type_ = MeasurementPackage::RADAR;

      iss >> pho;
      iss >> phi;
      iss >> pho_dot;
      measurement_package.measurement_values_ = Eigen::VectorXd(3);
      measurement_package.measurement_values_ << pho, phi, pho_dot;

      iss >> timestamp_r;
      measurement_package.timestamp_ = timestamp_r;

      // std::cout << pho << " " << phi << " " << pho_dot << " " << timestamp_r
      // << " ";
    }

    measurement_package_list.emplace_back(measurement_package);

    // get ground truth
    float x_gt, y_gt, vx_gt, vy_gt;
    iss >> x_gt;
    iss >> y_gt;
    iss >> vx_gt;
    iss >> vy_gt;

    gt_package.gt_values_ = Eigen::VectorXd(4);
    gt_package.gt_values_ << x_gt, y_gt, vx_gt, vy_gt;

    groundtruth_package_list.emplace_back(gt_package);

    // std::cout << x_gt << " " << y_gt << " " << vx_gt << " " << vy_gt
    // << std::endl;

    // test
    // break;
  }

  std::cout << "successed to load data." << std::endl;

  SensorFusion fuser;
  for (size_t i = 0; i < measurement_package_list.size(); ++i) {
    fuser.process(measurement_package_list[i]);
    Eigen::Vector4d x_out = fuser.kal_.getX();

    std::cout << "x: " << x_out(0) << " "
              << "y: " << x_out(1) << " "
              << "vx: " << x_out(2) << " "
              << "vy: " << x_out(3) << std::endl;
  }

  return 0;
}