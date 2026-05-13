/*
 * The vicon2gt project
 * Copyright (C) 2020 Patrick Geneva
 * Copyright (C) 2020 Guoquan Huang
 *
 * Ported to ROS2
 */

#include <Eigen/Eigen>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "meas/Interpolator.h"
#include "meas/Propagator.h"
#include "solver/ViconGraphSolver.h"

int main(int argc, char **argv) {

  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <config.yaml> [bag_path]" << std::endl;
    return EXIT_FAILURE;
  }

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("vicon2gt");

  node->declare_parameter("topic_imu", std::string(""));
  node->declare_parameter("topic_vicon", std::string(""));
  node->declare_parameter("dataset", std::string(""));

  auto rT1 = std::chrono::high_resolution_clock::now();

  std::string config_file = argv[1];
  std::cout << "Loading config from: " << config_file << std::endl;

  YAML::Node config = YAML::LoadFile(config_file);

  auto get_param = [&](const std::string &key, auto &val, auto default_val) {
    if (config[key]) {
      val = config[key].as<decltype(default_val)>();
    } else {
      val = default_val;
    }
  };

  std::string topic_imu, topic_vicon, path_to_bag, path_states, path_info, dataset;
  int state_freq;
  double bag_start, bag_durr;
  bool save_to_file, use_manual_sigmas;

  get_param("topic_imu", topic_imu, std::string("/imu0"));
  get_param("topic_vicon", topic_vicon, std::string("/vicon/odom"));
  get_param("path_bag", path_to_bag, std::string("bagfile.db3"));
  get_param("stats_path_states", path_states, std::string(""));
  get_param("stats_path_info", path_info, std::string(""));
  get_param("dataset", dataset, std::string(""));

  if (path_states.empty() && !dataset.empty()) {
    path_states = dataset + "_vicon2gt_states.csv";
    path_info = dataset + "_vicon2gt_info.txt";
  }
  if (path_states.empty()) {
    path_states = "gt_states.csv";
    path_info = "vicon2gt_info.txt";
  }
  get_param("state_freq", state_freq, 100);
  get_param("save_to_file", save_to_file, true);
  get_param("use_manual_sigmas", use_manual_sigmas, false);
  get_param("bag_start", bag_start, 0.0);
  get_param("bag_durr", bag_durr, -1.0);

  if (argc >= 3) {
    path_to_bag = argv[2];
  }
  path_to_bag += ".mcap";
  // Only override from rclcpp params if they have non-empty values
  try {
    rclcpp::Parameter param;
    if (node->get_parameter("topic_imu", param)) {
      std::string p = param.as_string();
      if (!p.empty())
        topic_imu = p;
    }
    if (node->get_parameter("topic_vicon", param)) {
      std::string p = param.as_string();
      if (!p.empty())
        topic_vicon = p;
    }
    if (node->get_parameter("dataset", param)) {
      std::string p = param.as_string();
      if (!p.empty()) {
        dataset = p;
        path_states = "./" + dataset + "_vicon2gt_states.csv";
        path_info = "./" + dataset + "_vicon2gt_info.txt";
      }
    }
  } catch (...) {
  }

  std::cout << "rosbag2 information..." << std::endl;
  std::cout << "    - bag path: " << path_to_bag << std::endl;
  std::cout << "    - state path: " << path_states << std::endl;
  std::cout << "    - info path: " << path_info << std::endl;
  std::cout << "    - save to file: " << (int)save_to_file << std::endl;
  std::cout << "    - use manual sigmas: " << (int)use_manual_sigmas << std::endl;
  std::cout << "    - state_freq: " << state_freq << std::endl;
  std::cout << "    - bag_start: " << bag_start << std::endl;
  std::cout << "    - bag_durr: " << bag_durr << std::endl;
  std::cout << "    - topic_imu: " << topic_imu << std::endl;
  std::cout << "    - topic_vicon: " << topic_vicon << std::endl;

  double sigma_w, sigma_wb, sigma_a, sigma_ab;
  get_param("gyroscope_noise_density", sigma_w, 1.6968e-04);
  get_param("accelerometer_noise_density", sigma_a, 2.0000e-3);
  get_param("gyroscope_random_walk", sigma_wb, 1.9393e-05);
  get_param("accelerometer_random_walk", sigma_ab, 3.0000e-3);

  std::string csv_vicon_path, csv_imu_path;
  get_param("csv_vicon", csv_vicon_path, std::string(""));
  get_param("csv_imu", csv_imu_path, std::string(""));

  std::cout << "CSV paths: csv_vicon='" << csv_vicon_path << "' csv_imu='" << csv_imu_path << "'" << std::endl;

  Eigen::Matrix3d R_q = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d R_p = Eigen::Matrix3d::Zero();
  std::vector<double> viconsigmas;
  std::vector<double> viconsigmas_default = {1e-4, 1e-4, 1e-4, 1e-5, 1e-5, 1e-5};
  get_param("vicon_sigmas", viconsigmas, viconsigmas_default);
  R_q(0, 0) = std::pow(viconsigmas.at(0), 2);
  R_q(1, 1) = std::pow(viconsigmas.at(1), 2);
  R_q(2, 2) = std::pow(viconsigmas.at(2), 2);
  R_p(0, 0) = std::pow(viconsigmas.at(3), 2);
  R_p(1, 1) = std::pow(viconsigmas.at(4), 2);
  R_p(2, 2) = std::pow(viconsigmas.at(5), 2);

  std::shared_ptr<Propagator> propagator = std::make_shared<Propagator>(sigma_w, sigma_wb, sigma_a, sigma_ab);
  std::shared_ptr<Interpolator> interpolator = std::make_shared<Interpolator>();

  int ct_imu = 0;
  int ct_vic = 0;
  double start_time = -1;
  double end_time = -1;
  double bag_start_time = 0;
  double bag_duration = 0;

  bool use_csv_input = !csv_vicon_path.empty() && !csv_imu_path.empty();

  if (use_csv_input) {
    std::cout << "loading from CSV files..." << std::endl;
    std::cout << "    - vicon CSV: " << csv_vicon_path << std::endl;
    std::cout << "    - imu CSV: " << csv_imu_path << std::endl;

    std::ifstream vic_csv(csv_vicon_path);
    if (!vic_csv.is_open()) {
      std::cerr << "Failed to open vicon CSV: " << csv_vicon_path << std::endl;
      return EXIT_FAILURE;
    }

    std::string line;
    std::getline(vic_csv, line);

    double last_vicon_time = -1;
    while (std::getline(vic_csv, line)) {
      if (line.empty() || line[0] == '#')
        continue;

      std::stringstream ss(line);
      double ts, px, py, pz, qx, qy, qz, qw;
      char comma;
      ss >> ts >> comma >> px >> comma >> py >> comma >> pz >> comma >> qx >> comma >> qy >> comma >> qz >> comma >> qw;

      Eigen::Vector4d q;
      Eigen::Vector3d p;
      q << qx, qy, qz, qw;
      p << px, py, pz;

      interpolator->feed_pose(ts, q, p, R_q, R_p);

      if (start_time == -1 || ts < start_time)
        start_time = ts;
      end_time = ts;
      last_vicon_time = ts;
      ct_vic++;
    }
    vic_csv.close();

    std::ifstream imu_csv(csv_imu_path);
    if (!imu_csv.is_open()) {
      std::cerr << "Failed to open IMU CSV: " << csv_imu_path << std::endl;
      return EXIT_FAILURE;
    }

    std::getline(imu_csv, line);
    while (std::getline(imu_csv, line)) {
      if (line.empty() || line[0] == '#')
        continue;

      std::stringstream ss(line);
      double ts, wx, wy, wz, ax, ay, az;
      char comma;
      ss >> ts >> comma >> wx >> comma >> wy >> comma >> wz >> comma >> ax >> comma >> ay >> comma >> az;

      Eigen::Vector3d wm, am;
      wm << wx, wy, wz;
      am << ax, ay, az;

      propagator->feed_imu(ts, wm, am);
      ct_imu++;
    }
    imu_csv.close();

    bag_start_time = start_time;
    bag_duration = end_time - start_time;
    bag_start = 0.0;
    bag_durr = bag_duration;

    std::cout << "    - time start = " << std::fixed << std::setprecision(6) << start_time << std::endl;
    std::cout << "    - time end   = " << end_time << std::endl;
    std::cout << "    - duration   = " << bag_duration << " (secs)" << std::endl;
    std::cout << "    - number imu   = " << ct_imu << std::endl;
    std::cout << "    - number vicon = " << ct_vic << std::endl;
  } else {
    std::cout << "loading rosbag2..." << std::endl;

    rosbag2_cpp::Reader reader;
    try {
      reader.open(path_to_bag);
    } catch (const std::exception &e) {
      std::cerr << "Failed to open bag: " << e.what() << std::endl;
      return EXIT_FAILURE;
    }

    auto metadata = reader.get_metadata();
    bag_start_time = metadata.starting_time.time_since_epoch().count() / 1e9;
    bag_duration = metadata.duration.count() / 1e9;
    double time_init = bag_start_time + bag_start;
    double time_finish = (bag_durr < 0) ? (bag_start_time + bag_duration) : (time_init + bag_durr);

    std::cout << "    - time start = " << std::fixed << std::setprecision(6) << time_init << std::endl;
    std::cout << "    - time end   = " << time_finish << std::endl;
    std::cout << "    - duration   = " << (time_finish - time_init) << " (secs)" << std::endl;

    double last_vicon_time = -1;
    double max_vicon_lost_time = 1.0;

    auto warn_amount_vicon_rate = [&](double timestamp, double timestamp_last) {
      double vicon_dt = timestamp - timestamp_last;
      if (last_vicon_time == -1 || vicon_dt < max_vicon_lost_time)
        return;
      double dist_from_start = timestamp_last - time_init;
      std::cout << "WARNING: over " << vicon_dt << " seconds of no vicon!! (starting " << dist_from_start << " sec into bag)" << std::endl;
    };

    while (reader.has_next()) {
      if (!rclcpp::ok())
        break;

      auto msg = reader.read_next();

      double timestamp = msg->time_stamp / 1e9;

      if (timestamp < time_init || timestamp > time_finish) {
        continue;
      }

      if (msg->topic_name == topic_imu) {
        auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
        rclcpp::Serialization<sensor_msgs::msg::Imu> serialization;
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
        serialization.deserialize_message(&serialized_msg, imu_msg.get());

        timestamp = imu_msg->header.stamp.sec + imu_msg->header.stamp.nanosec * 1e-9;

        Eigen::Vector3d wm, am;
        wm << imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z;
        am << imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z;
        propagator->feed_imu(timestamp, wm, am);
        ct_imu++;
      } else if (msg->topic_name == topic_vicon) {
        bool processed = false;

        auto transform_msg = std::make_shared<geometry_msgs::msg::TransformStamped>();
        if (msg->serialized_data) {
          try {
            rclcpp::Serialization<geometry_msgs::msg::TransformStamped> transform_serialization;
            rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
            transform_serialization.deserialize_message(&serialized_msg, transform_msg.get());

            timestamp = transform_msg->header.stamp.sec + transform_msg->header.stamp.nanosec * 1e-9;

            Eigen::Vector4d q;
            Eigen::Vector3d p;
            q << transform_msg->transform.rotation.x, transform_msg->transform.rotation.y, transform_msg->transform.rotation.z,
                transform_msg->transform.rotation.w;
            p << transform_msg->transform.translation.x, transform_msg->transform.translation.y, transform_msg->transform.translation.z;

            interpolator->feed_pose(timestamp, q, p, R_q, R_p);
            processed = true;

            if (start_time == -1) {
              start_time = timestamp;
            }
            end_time = timestamp;
            warn_amount_vicon_rate(timestamp, last_vicon_time);
            last_vicon_time = timestamp;
            ct_vic++;
          } catch (...) {
          }
        }

        if (!processed) {
          auto odom_msg = std::make_shared<nav_msgs::msg::Odometry>();
          if (msg->serialized_data) {
            try {
              rclcpp::Serialization<nav_msgs::msg::Odometry> odom_serialization;
              rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
              odom_serialization.deserialize_message(&serialized_msg, odom_msg.get());

              Eigen::Vector4d q;
              Eigen::Vector3d p;
              q << odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z,
                  odom_msg->pose.pose.orientation.w;
              p << odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z;

              Eigen::Matrix<double, 6, 6> pose_cov = Eigen::Matrix<double, 6, 6>::Zero();
              if (!use_manual_sigmas && !odom_msg->pose.covariance.empty()) {
                for (size_t c = 0; c < 6; c++) {
                  for (size_t r = 0; r < 6; r++) {
                    pose_cov(r, c) = odom_msg->pose.covariance[6 * c + r];
                  }
                }
              } else {
                pose_cov.block(3, 3, 3, 3) = R_q;
                pose_cov.block(0, 0, 3, 3) = R_p;
              }
              interpolator->feed_pose(timestamp, q, p, pose_cov.block(3, 3, 3, 3), pose_cov.block(0, 0, 3, 3));
              processed = true;

              if (start_time == -1) {
                start_time = timestamp;
              }
              end_time = timestamp;
              warn_amount_vicon_rate(timestamp, last_vicon_time);
              last_vicon_time = timestamp;
              ct_vic++;
            } catch (...) {
            }
          }
        }
      }
    }

    reader.close();
  }

  auto rT3 = std::chrono::high_resolution_clock::now();
  auto load_duration = std::chrono::duration_cast<std::chrono::microseconds>(rT3 - rT1).count();
  std::cout << "[TIME]: " << load_duration * 1e-6 << " to preprocess" << std::endl;

  int ct_cam = 0;
  std::vector<double> timestamp_cameras;
  if (start_time != -1 && end_time != -1 && start_time < end_time) {
    double temp_time = start_time;
    while (temp_time < end_time) {
      timestamp_cameras.push_back(temp_time);
      temp_time += 1.0 / (double)state_freq;
      ct_cam++;
    }
  }

  std::cout << "done loading the rosbag..." << std::endl;
  std::cout << "    - number imu   = " << ct_imu << std::endl;
  std::cout << "    - number cam   = " << ct_cam << std::endl;
  std::cout << "    - number vicon = " << ct_vic << std::endl;

  if (ct_imu == 0 || ct_cam == 0 || ct_vic == 0) {
    std::cerr << "Not enough data to optimize with!" << std::endl;
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  ViconGraphSolver solver(config_file, propagator, interpolator, timestamp_cameras);
  solver.build_and_solve();

  solver.visualize();

  if (save_to_file) {
    solver.write_to_file(path_states, path_info);
  }

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}