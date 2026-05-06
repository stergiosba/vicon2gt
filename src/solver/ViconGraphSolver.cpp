/*
 * The vicon2gt project
 * Copyright (C) 2020 Patrick Geneva
 * Copyright (C) 2020 Guoquan Huang
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#include "ViconGraphSolver.h"
#include <iomanip>
#include <iostream>
#include <yaml-cpp/yaml.h>

ViconGraphSolver::ViconGraphSolver(const std::string &config_file, std::shared_ptr<Propagator> propagator,
                                   std::shared_ptr<Interpolator> interpolator, std::vector<double> timestamp_cameras)
    : config_file_(config_file) {

  this->propagator = propagator;
  this->interpolator = interpolator;
  this->timestamp_cameras = timestamp_cameras;

  this->graph = new gtsam::NonlinearFactorGraph();
  this->config = std::make_shared<GtsamConfig>();

  YAML::Node config_yaml;
  try {
    config_yaml = YAML::LoadFile(config_file);
  } catch (const std::exception &e) {
    std::cerr << "[VICON-GRAPH]: Failed to load config file: " << config_file << std::endl;
    std::cerr << "Error: " << e.what() << std::endl;
    std::exit(EXIT_FAILURE);
  }

  auto load_vector = [&](const std::string &key, std::vector<double> &vec, const std::vector<double> &default_val) {
    if (config_yaml[key] && config_yaml[key].IsSequence()) {
      vec = config_yaml[key].as<std::vector<double>>();
    } else {
      vec = default_val;
    }
  };

  auto load_double = [&](const std::string &key, double &val, double default_val) {
    if (config_yaml[key]) {
      val = config_yaml[key].as<double>();
    } else {
      val = default_val;
    }
  };

  auto load_bool = [&](const std::string &key, bool &val, bool default_val) {
    if (config_yaml[key]) {
      val = config_yaml[key].as<bool>();
    } else {
      val = default_val;
    }
  };

  auto load_int = [&](const std::string &key, int &val, int default_val) {
    if (config_yaml[key]) {
      val = config_yaml[key].as<int>();
    } else {
      val = default_val;
    }
  };

  std::vector<double> R_GtoV;
  std::vector<double> R_GtoV_default = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  load_vector("R_GtoV", R_GtoV, R_GtoV_default);
  init_R_GtoV << R_GtoV.at(0), R_GtoV.at(1), R_GtoV.at(2), R_GtoV.at(3), R_GtoV.at(4), R_GtoV.at(5), R_GtoV.at(6), R_GtoV.at(7),
      R_GtoV.at(8);

  load_double("gravity_magnitude", gravity_magnitude, 9.81);

  std::vector<double> R_BtoI;
  std::vector<double> R_BtoI_default = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  load_vector("R_BtoI", R_BtoI, R_BtoI_default);
  init_R_BtoI << R_BtoI.at(0), R_BtoI.at(1), R_BtoI.at(2), R_BtoI.at(3), R_BtoI.at(4), R_BtoI.at(5), R_BtoI.at(6), R_BtoI.at(7),
      R_BtoI.at(8);

  std::vector<double> p_BinI;
  std::vector<double> p_BinI_default = {0.0, 0.0, 0.0};
  load_vector("p_BinI", p_BinI, p_BinI_default);
  init_p_BinI << p_BinI.at(0), p_BinI.at(1), p_BinI.at(2);

  load_double("toff_imu_to_vicon", init_toff_imu_to_vicon, 0.0);

  std::cout << "init_R_GtoV:" << std::endl << init_R_GtoV << std::endl;
  std::cout << "init_R_BtoI:" << std::endl << init_R_BtoI << std::endl;
  std::cout << "init_p_BinI:" << std::endl << init_p_BinI.transpose() << std::endl;
  std::cout << "init_toff_imu_to_vicon:" << std::endl << init_toff_imu_to_vicon << std::endl;

  load_bool("estimate_toff_vicon_to_imu", config->estimate_vicon_imu_toff, config->estimate_vicon_imu_toff);
  load_bool("estimate_ori_vicon_to_imu", config->estimate_vicon_imu_ori, config->estimate_vicon_imu_ori);
  load_bool("estimate_pos_vicon_to_imu", config->estimate_vicon_imu_pos, config->estimate_vicon_imu_pos);

  load_int("num_loop_relin", num_loop_relin, 0);

  std::cout << "estimate_toff_vicon_to_imu: " << (int)config->estimate_vicon_imu_toff << std::endl;
  std::cout << "estimate_ori_vicon_to_imu: " << (int)config->estimate_vicon_imu_ori << std::endl;
  std::cout << "estimate_pos_vicon_to_imu: " << (int)config->estimate_vicon_imu_pos << std::endl;
  std::cout << "num_loop_relin: " << num_loop_relin << std::endl;
}

void ViconGraphSolver::build_and_solve() {

  if (timestamp_cameras.empty()) {
    std::cerr << "[VICON-GRAPH]: Camera timestamp vector empty!!!!" << std::endl;
    std::cerr << "[VICON-GRAPH]: Make sure your camera topic is correct..." << std::endl;
    std::exit(EXIT_FAILURE);
  }

  if (interpolator->get_raw_poses().size() < 2) {
    std::cerr << "[VICON-GRAPH]: Not enough vicon poses to optimize with..." << std::endl;
    std::cerr << "[VICON-GRAPH]: Make sure your vicon topic is correct..." << std::endl;
    std::exit(EXIT_FAILURE);
  }

  std::cout << "cleaning camera timestamps" << std::endl;
  int ct_remove_imu = 0;
  int ct_remove_before = 0;
  int ct_remove_after = 0;
  double vicon_first_time_inI = interpolator->get_time_min() + init_toff_imu_to_vicon + 2 * TIME_OFFSET;
  double vicon_last_time_inI = interpolator->get_time_max() + init_toff_imu_to_vicon - 2 * TIME_OFFSET;
  auto it0 = timestamp_cameras.begin();
  while (it0 != timestamp_cameras.end()) {
    if (!propagator->has_bounding_imu(*it0)) {
      std::cout << "    - deleted cam time " << std::fixed << std::setprecision(9) << *it0 << " with no IMU" << std::endl;
      it0 = timestamp_cameras.erase(it0);
      ct_remove_imu++;
    } else if ((*it0) < vicon_first_time_inI) {
      std::cout << "    - deleted cam time " << *it0 << " before first vicon" << std::endl;
      it0 = timestamp_cameras.erase(it0);
      ct_remove_before++;
    } else if ((*it0) > vicon_last_time_inI) {
      std::cout << "    - deleted cam time " << *it0 << " after last vicon" << std::endl;
      it0 = timestamp_cameras.erase(it0);
      ct_remove_after++;
    } else {
      it0++;
    }
  }
  std::cout << "removed " << ct_remove_imu << " imu invalid, " << ct_remove_before << " invalid before vicon, " << ct_remove_after
            << " invalid after vicon" << std::endl;

  if (timestamp_cameras.empty()) {
    std::cerr << "[VICON-GRAPH]: All camera timestamps where out of the range of the IMU measurements." << std::endl;
    std::cerr << "[VICON-GRAPH]: Make sure your vicon and imu topics are correct..." << std::endl;
    std::exit(EXIT_FAILURE);
  }

  map_states.clear();
  values.clear();

  for (size_t i = 0; i < timestamp_cameras.size(); i++) {
    map_states.insert({timestamp_cameras.at(i), i});
  }

  for (int i = 0; i <= num_loop_relin; i++) {

    build_problem(i == 0);

    optimize_problem();

    values = values_result;

    auto duration_build = (rT2 - rT1).total_microseconds();
    auto duration_optimize = (rT3 - rT2).total_microseconds();
    auto duration_total = (rT3 - rT1).total_microseconds();
    std::cout << "[TIME]: " << duration_build * 1e-6 << " to build" << std::endl;
    std::cout << "[TIME]: " << duration_optimize * 1e-6 << " to optimize" << std::endl;
    std::cout << "[TIME]: " << duration_total * 1e-6 << " total (loop " << i << ")" << std::endl;

    visualize();
  }

  std::cout << std::endl << "======================================" << std::endl;
  std::cout << "state_0: " << std::endl << values_result.at<JPLNavState>(X(map_states[timestamp_cameras.at(0)])) << std::endl;
  std::cout << "state_N: " << std::endl
            << values_result.at<JPLNavState>(X(map_states[timestamp_cameras.at(timestamp_cameras.size() - 1)])) << std::endl;
  std::cout << "R_BtoI: " << std::endl << quat_2_Rot(values_result.at<JPLQuaternion>(C(0)).q()) << std::endl << std::endl;
  std::cout << "p_BinI: " << std::endl << values_result.at<Vector3>(C(1)) << std::endl << std::endl;
  std::cout << "R_GtoV: " << std::endl << values_result.at<RotationXY>(G(0)).rot() << std::endl << std::endl;
  std::cout << "t_off_vicon_to_imu: " << std::endl << values_result.at<Vector1>(T(0)) << std::endl << std::endl;
  std::cout << "======================================" << std::endl << std::endl;
}

void ViconGraphSolver::write_to_file(std::string csvfilepath, std::string infofilepath) {

  std::cout << "saving states and info to file" << std::endl;
  std::cout << "    - state file: " << csvfilepath << std::endl;
  std::cout << "    - info file: " << infofilepath << std::endl;

  namespace fs = boost::filesystem;

  fs::path p1(csvfilepath);
  fs::path p1_parent = p1.parent_path();
  if (!p1_parent.empty() && fs::exists(p1_parent)) {
    if (fs::exists(csvfilepath)) {
      fs::remove(csvfilepath);
      std::cout << "    - old state file found, deleted..." << std::endl;
    }
  } else {
    std::cout << "    - using current directory for state file" << std::endl;
    csvfilepath = "./" + fs::path(csvfilepath).filename().string();
  }

  fs::path p2(infofilepath);
  fs::path p2_parent = p2.parent_path();
  if (!p2_parent.empty() && fs::exists(p2_parent)) {
    if (fs::exists(infofilepath)) {
      fs::remove(infofilepath);
      std::cout << "    - old info file found, deleted..." << std::endl;
    }
  } else {
    std::cout << "    - using current directory for info file" << std::endl;
    infofilepath = "./" + fs::path(infofilepath).filename().string();
  }

  std::ofstream of_state;
  of_state.open(csvfilepath, std::ofstream::out | std::ofstream::app);
  of_state << "#time(ns),px,py,pz,qw,qx,qy,qz,vx,vy,vz,bwx,bwy,bwz,bax,bay,baz" << std::endl;

  Eigen::Matrix3d R_GtoV = values_result.at<RotationXY>(G(0)).rot();
  Eigen::Vector4d q_GtoV = rot_2_quat(R_GtoV);
  Eigen::Matrix3d R_BtoI = quat_2_Rot(values_result.at<JPLQuaternion>(C(0)).q());
  Eigen::Vector3d p_BinI = values_result.at<Vector3>(C(1));

  bool output_vicon_frame = false;
  for (size_t i = 0; i < timestamp_cameras.size(); i++) {
    JPLNavState state = values_result.at<JPLNavState>(X(map_states[timestamp_cameras.at(i)]));

    Eigen::Vector3d p_IiinG = state.p();
    Eigen::Vector3d v_IiinG = state.v();
    Eigen::Vector4d q_GtoIi = state.q();

    if (output_vicon_frame) {
      p_IiinG = R_GtoV.transpose() * state.p();
      v_IiinG = R_GtoV.transpose() * state.v();
      q_GtoIi = quat_multiply(state.q(), q_GtoV);

      Eigen::Vector3d p_BinV = p_IiinG + R_BtoI.transpose() * p_BinI;
      p_IiinG = p_BinV;
      v_IiinG = R_GtoV.transpose() * R_BtoI.transpose() * state.v();
      q_GtoIi = quat_multiply(rot_2_quat(R_BtoI.transpose()), q_GtoIi);
    }

    of_state << std::setprecision(20) << std::floor(1e9 * timestamp_cameras.at(i)) << "," << std::setprecision(6) << p_IiinG(0) << ","
             << p_IiinG(1) << "," << p_IiinG(2) << "," << q_GtoIi(3) << "," << q_GtoIi(0) << "," << q_GtoIi(1) << "," << q_GtoIi(2) << ","
             << v_IiinG(0) << "," << v_IiinG(1) << "," << v_IiinG(2) << "," << state.bg()(0) << "," << state.bg()(1) << "," << state.bg()(2)
             << "," << state.ba()(0) << "," << state.ba()(1) << "," << state.ba()(2) << std::endl;
  }
  of_state.close();

  std::ofstream of_info;
  of_info.open(infofilepath, std::ofstream::out | std::ofstream::app);
  of_info << "R_BtoI: " << std::endl << quat_2_Rot(values_result.at<JPLQuaternion>(C(0)).q()) << std::endl << std::endl;
  of_info << "q_BtoI: " << std::endl << values_result.at<JPLQuaternion>(C(0)).q() << std::endl << std::endl;
  of_info << "p_BinI: " << std::endl << values_result.at<Vector3>(C(1)) << std::endl << std::endl;
  of_info << "R_GtoV: " << std::endl << values_result.at<RotationXY>(G(0)).rot() << std::endl << std::endl;
  of_info << "R_GtoV (thetax, thetay): " << std::endl;
  of_info << values_result.at<RotationXY>(G(0)).thetax() << " " << values_result.at<RotationXY>(G(0)).thetax() << std::endl << std::endl;
  of_info << "gravity norm: " << std::endl << gravity_magnitude << std::endl << std::endl;
  of_info << "t_off_vicon_to_imu: " << std::endl << values_result.at<Vector1>(T(0)) << std::endl << std::endl;
  of_info.close();
}

void ViconGraphSolver::visualize() { std::cout << "Visualization: CSV output only (ROS visualization disabled in ROS2 port)" << std::endl; }

void ViconGraphSolver::get_imu_poses(std::vector<double> &times, std::vector<Eigen::Matrix<double, 10, 1>> &poses) {

  times.clear();
  poses.clear();

  for (size_t i = 0; i < timestamp_cameras.size(); i++) {
    JPLNavState state = values_result.at<JPLNavState>(X(map_states[timestamp_cameras.at(i)]));

    Eigen::Matrix<double, 10, 1> pose;
    pose << state.q(), state.p(), state.v();
    times.push_back(timestamp_cameras.at(i));
    poses.push_back(pose);
  }
}

void ViconGraphSolver::get_calibration(double &toff, Eigen::Matrix3d &R_BtoI, Eigen::Vector3d &p_BinI, Eigen::Matrix3d &R_GtoV) {

  R_BtoI = quat_2_Rot(values_result.at<JPLQuaternion>(C(0)).q());
  p_BinI = values_result.at<Vector3>(C(1));

  R_GtoV = values_result.at<RotationXY>(G(0)).rot();

  toff = values_result.at<Vector1>(T(0)).matrix()(0);
}

void ViconGraphSolver::build_problem(bool init_states) {

  rT1 = boost::posix_time::microsec_clock::local_time();

  std::cout << "[BUILD]: building the graph (might take a while)" << std::endl;
  graph->erase(graph->begin(), graph->end());

  if (init_states) {
    values.insert(C(0), JPLQuaternion(rot_2_quat(init_R_BtoI)));
    values.insert(C(1), Vector3(init_p_BinI));
    Eigen::Vector3d rpy = rot2rpy(init_R_GtoV);
    values.insert(G(0), RotationXY(rpy(0), rpy(1)));
  }
  std::cout << "[BUILD]: initial R_GtoV roll pitch " << values.at<RotationXY>(G(0)).thetax() << ", " << values.at<RotationXY>(G(0)).thetay()
            << std::endl;

  if (init_states) {
    Vector1 temp;
    temp(0) = init_toff_imu_to_vicon;
    values.insert(T(0), temp);
  }
  std::cout << "[BUILD]: current time offset is " << values.at<Vector1>(T(0))(0) << std::endl;

  auto it1 = timestamp_cameras.begin();
  while (it1 != timestamp_cameras.end()) {

    double timestamp_inI = *it1;
    double time_from_start = timestamp_inI - timestamp_cameras.at(0);
    double timestamp_inV = timestamp_inI - values.at<Vector1>(T(0))(0);

    Eigen::Vector4d q_VtoB, q_VtoB0, q_VtoB2;
    Eigen::Vector3d p_BinV, p_B0inV, p_B2inV;
    Eigen::Matrix<double, 6, 6> R_vicon;
    bool has_vicon1 = interpolator->get_pose(timestamp_inV - TIME_OFFSET, q_VtoB0, p_B0inV, R_vicon);
    bool has_vicon2 = interpolator->get_pose(timestamp_inV + TIME_OFFSET, q_VtoB2, p_B2inV, R_vicon);
    bool has_vicon3 = interpolator->get_pose(timestamp_inV, q_VtoB, p_BinV, R_vicon);

    if (!has_vicon1 || !has_vicon2 || !has_vicon3) {
      std::cout << "    - skipping camera time " << std::fixed << std::setprecision(9) << timestamp_inI << " - " << time_from_start
                << " from beginning (no vicon pose found)" << std::endl;
      if (values.find(X(map_states[timestamp_inI])) != values.end()) {
        values.erase(X(map_states[timestamp_inI]));
      }
      it1 = timestamp_cameras.erase(it1);
      continue;
    }

    if (std::isnan(R_vicon.norm()) || std::isnan(R_vicon.inverse().norm())) {
      std::cout << "    - skipping camera time " << timestamp_inI << " - " << time_from_start
                << " from beginning (R.norm = " << R_vicon.norm() << " | Rinv.norm = " << R_vicon.inverse().norm() << ")" << std::endl;
      if (values.find(X(map_states[timestamp_inI])) != values.end()) {
        values.erase(X(map_states[timestamp_inI]));
      }
      it1 = timestamp_cameras.erase(it1);
      continue;
    }

    if (init_states) {

      // std::cout << "      [DEBUG] timestamp_inI=" << timestamp_inI << std::endl;
      // std::cout << "      [DEBUG] p_BinV=" << p_BinV.transpose() << std::endl;
      // std::cout << "      [DEBUG] init_p_BinI=" << init_p_BinI.transpose() << std::endl;
      // std::cout << "      [DEBUG] R_BtoI^T * p_BinI=" << (init_R_BtoI.transpose() * init_p_BinI).transpose() << std::endl;

      Eigen::Vector4d q_VtoI = quat_multiply(rot_2_quat(init_R_BtoI), q_VtoB);
      Eigen::Vector3d p_IinV = p_BinV - quat_2_Rot(Inv(q_VtoB)) * init_R_BtoI.transpose() * init_p_BinI;

      Eigen::Vector3d p_I0inV = p_B0inV - quat_2_Rot(Inv(q_VtoB0)) * init_R_BtoI.transpose() * init_p_BinI;
      Eigen::Vector3d p_I2inV = p_B2inV - quat_2_Rot(Inv(q_VtoB2)) * init_R_BtoI.transpose() * init_p_BinI;
      Eigen::Vector3d v_IinV = (p_I2inV - p_I0inV) / (2 * TIME_OFFSET);

      Eigen::Vector3d bg = Eigen::Vector3d::Zero();
      Eigen::Vector3d ba = Eigen::Vector3d::Zero();

      JPLNavState imu_state(timestamp_inI, q_VtoI, bg, v_IinV, ba, p_IinV);
      values.insert(X(map_states[timestamp_inI]), imu_state);
    }

    MeasBased_ViconPoseTimeoffsetFactor factor_vicon(X(map_states[timestamp_inI]), C(0), C(1), T(0), interpolator, config);
    graph->add(factor_vicon);

    if (it1 == timestamp_cameras.begin()) {
      it1++;
      continue;
    }

    double time0 = *(it1 - 1);
    double time1 = *(it1);

    Bias3 bg = values.at<JPLNavState>(X(map_states[time0])).bg();
    Bias3 ba = values.at<JPLNavState>(X(map_states[time0])).ba();

    CpiV1 preint(0, 0, 0, 0, true);
    bool has_imu = propagator->propagate(time0, time1, bg, ba, preint);
    if (!has_imu || preint.DT != (time1 - time0)) {
      std::cerr << "unable to get IMU readings, invalid preint" << std::endl;
      std::cerr << "preint.DT = " << preint.DT << " | (time1-time0) = " << time1 - time0 << std::endl;
      std::exit(EXIT_FAILURE);
    }

    if (std::isnan(preint.P_meas.norm()) || std::isnan(preint.P_meas.inverse().norm())) {
      std::cerr << "R_imu is NAN | R.norm = " << preint.P_meas.norm() << " | Rinv.norm = " << preint.P_meas.inverse().norm() << std::endl;
      std::cerr << "THIS SHOULD NEVER HAPPEN!" << std::endl;
      std::exit(EXIT_FAILURE);
    }

    ImuFactorCPIv1 factor_imu(X(map_states[time0]), X(map_states[time1]), G(0), preint.P_meas, preint.DT, gravity_magnitude,
                              preint.alpha_tau, preint.beta_tau, preint.q_k2tau, preint.b_a_lin, preint.b_w_lin, preint.J_q, preint.J_b,
                              preint.J_a, preint.H_b, preint.H_a);
    graph->add(factor_imu);

    it1++;
  }
  rT2 = boost::posix_time::microsec_clock::local_time();
}

void ViconGraphSolver::optimize_problem() {

  std::cout << "[VICON-GRAPH]: graph factors - " << (int)graph->nrFactors() << std::endl;
  std::cout << "[VICON-GRAPH]: graph nodes - " << (int)graph->keys().size() << std::endl;

  LevenbergMarquardtParams opti_config;
  opti_config.verbosity = NonlinearOptimizerParams::Verbosity::TERMINATION;
  opti_config.orderingType = Ordering::OrderingType::METIS;
  opti_config.absoluteErrorTol = 1e-30;
  opti_config.relativeErrorTol = 1e-30;
  opti_config.lambdaUpperBound = 1e20;
  opti_config.maxIterations = 30;
  LevenbergMarquardtOptimizer optimizer(*graph, values, opti_config);

  std::cout << "[VICON-GRAPH]: begin optimization" << std::endl;
  values_result = optimizer.optimize();
  std::cout << "[VICON-GRAPH]: done optimization (" << (int)optimizer.iterations() << " iterations)!" << std::endl;
  rT3 = boost::posix_time::microsec_clock::local_time();
}