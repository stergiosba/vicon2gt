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

#include <Eigen/Eigen>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "meas/Interpolator.h"
#include "meas/Propagator.h"
#include "sim/Simulator.h"
#include "solver/ViconGraphSolver.h"
#include "utils/stats.h"

int main(int argc, char **argv) {

  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <config.yaml>" << std::endl;
    return EXIT_FAILURE;
  }

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

  bool save_to_file;
  std::string path_states, path_states_gt, path_info;
  int state_freq;

  get_param("stats_path_states", path_states, std::string("states.csv"));
  get_param("stats_path_states_gt", path_states_gt, std::string("gt.csv"));
  get_param("stats_path_info", path_info, std::string("vicon2gt_info.txt"));
  get_param("save_to_file", save_to_file, false);
  get_param("state_freq", state_freq, 100);

  std::cout << "save path information..." << std::endl;
  std::cout << "    - state path: " << path_states << std::endl;
  std::cout << "    - info path: " << path_info << std::endl;
  std::cout << "    - save to file: " << (int)save_to_file << std::endl;
  std::cout << "    - state_freq: " << state_freq << std::endl;

  SimulatorParams params;

  get_param("sim_traj_path", params.sim_traj_path, params.sim_traj_path);
  get_param("sim_freq_imu", params.sim_freq_imu, params.sim_freq_imu);
  get_param("sim_freq_cam", params.sim_freq_cam, params.sim_freq_cam);
  get_param("sim_freq_vicon", params.sim_freq_vicon, params.sim_freq_vicon);
  get_param("sim_seed", params.seed, params.seed);

  double sigma_w, sigma_wb, sigma_a, sigma_ab;
  get_param("gyroscope_noise_density", sigma_w, 1.6968e-04);
  get_param("accelerometer_noise_density", sigma_a, 2.0000e-3);
  get_param("gyroscope_random_walk", sigma_wb, 1.9393e-05);
  get_param("accelerometer_random_walk", sigma_ab, 3.0000e-3);

  Eigen::Matrix3d R_q = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d R_p = Eigen::Matrix3d::Zero();
  std::vector<double> viconsigmas;
  std::vector<double> viconsigmas_default = {1e-3, 1e-3, 1e-3, 1e-2, 1e-2, 1e-2};
  get_param("vicon_sigmas", viconsigmas, viconsigmas_default);
  R_q(0, 0) = std::pow(viconsigmas.at(0), 2);
  R_q(1, 1) = std::pow(viconsigmas.at(1), 2);
  R_q(2, 2) = std::pow(viconsigmas.at(2), 2);
  R_p(0, 0) = std::pow(viconsigmas.at(3), 2);
  R_p(1, 1) = std::pow(viconsigmas.at(4), 2);
  R_p(2, 2) = std::pow(viconsigmas.at(5), 2);
  params.sigma_vicon_pose << viconsigmas.at(0), viconsigmas.at(1), viconsigmas.at(2), viconsigmas.at(3), viconsigmas.at(4),
      viconsigmas.at(5);

  get_param("gravity_magnitude", params.gravity_magnitude, 9.81);

  std::shared_ptr<Simulator> sim = std::make_shared<Simulator>(params);

  std::shared_ptr<Propagator> propagator = std::make_shared<Propagator>(sigma_w, sigma_wb, sigma_a, sigma_ab);
  std::shared_ptr<Interpolator> interpolator = std::make_shared<Interpolator>();

  int ct_imu = 0;
  int ct_vic = 0;
  double start_time = -1;
  double end_time = -1;

  while (sim->ok()) {

    double time_imu;
    Eigen::Vector3d wm, am;
    if (sim->get_next_imu(time_imu, wm, am)) {
      propagator->feed_imu(time_imu, wm, am);
      ct_imu++;
    }

    double time_cam;
    sim->get_next_cam(time_cam);

    double time_vicon;
    Eigen::Vector4d q_VtoB;
    Eigen::Vector3d p_BinV;
    if (sim->get_next_vicon(time_vicon, q_VtoB, p_BinV)) {
      interpolator->feed_pose(time_vicon, q_VtoB, p_BinV, R_q, R_p);
      ct_vic++;
      if (start_time == -1) {
        start_time = time_vicon;
      }
      if (start_time != -1) {
        end_time = time_vicon;
      }
    }
  }

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

  std::cout << "done loading simulation..." << std::endl;
  std::cout << "    - number imu   = " << ct_imu << std::endl;
  std::cout << "    - number cam   = " << ct_cam << std::endl;
  std::cout << "    - number vicon = " << ct_vic << std::endl;

  ViconGraphSolver solver(config_file, propagator, interpolator, timestamp_cameras);
  solver.build_and_solve();

  solver.visualize();

  if (save_to_file) {
    solver.write_to_file(path_states, path_info);
  }

  namespace fs = boost::filesystem;
  if (save_to_file) {
    solver.write_to_file(path_states, path_info);

    if (fs::exists(path_states_gt)) {
      fs::remove(path_states_gt);
      std::cout << "    - old state file found, deleted..." << std::endl;
    }
    fs::path p1(path_states_gt);
    fs::create_directories(p1.parent_path());
  }

  std::ofstream of_state;
  if (save_to_file) {
    of_state.open(path_states_gt, std::ofstream::out | std::ofstream::app);
    of_state << "#time(ns),px,py,pz,qw,qx,qy,qz,vx,vy,vz,bwx,bwy,bwz,bax,bay,baz" << std::endl;
  }

  std::vector<double> times;
  std::vector<Eigen::Matrix<double, 10, 1>> poses;
  solver.get_imu_poses(times, poses);

  Stats err_ori, err_pos, err_vel;
  for (size_t i = 0; i < times.size(); i++) {

    Eigen::Matrix<double, 17, 1> gt_state;
    sim->get_state_in_vicon(times.at(i), gt_state);
    Eigen::Matrix<double, 10, 1> est_state = poses.at(i);

    double ori = 2.0 * (quat_multiply(gt_state.block(1, 0, 4, 1), Inv(est_state.block(0, 0, 4, 1)))).block(0, 0, 3, 1).norm();
    double pose = (est_state.block(4, 0, 3, 1) - gt_state.block(5, 0, 3, 1)).norm();
    double vel = (est_state.block(7, 0, 3, 1) - gt_state.block(8, 0, 3, 1)).norm();

    err_ori.timestamps.push_back(times.at(i));
    err_ori.values.push_back(180.0 / M_PI * ori);
    err_pos.timestamps.push_back(times.at(i));
    err_pos.values.push_back(pose);
    err_vel.timestamps.push_back(times.at(i));
    err_vel.values.push_back(vel);

    if (of_state.is_open()) {
      Eigen::Vector4d q_GtoV = rot_2_quat(sim->get_params().R_GtoV);
      Eigen::Vector4d q_GtoIi = quat_multiply(gt_state.block(1, 0, 4, 1), q_GtoV);
      Eigen::Vector3d p_IiinG = sim->get_params().R_GtoV.transpose() * gt_state.block(5, 0, 3, 1);
      Eigen::Vector3d v_IiinG = sim->get_params().R_GtoV.transpose() * gt_state.block(8, 0, 3, 1);
      of_state << std::setprecision(20) << std::floor(1e9 * times.at(i)) << "," << std::setprecision(6) << p_IiinG(0) << "," << p_IiinG(1)
               << "," << p_IiinG(2) << "," << q_GtoIi(3) << "," << q_GtoIi(0) << "," << q_GtoIi(1) << "," << q_GtoIi(2) << "," << v_IiinG(0)
               << "," << v_IiinG(1) << "," << v_IiinG(2) << "," << gt_state(11) << "," << gt_state(12) << "," << gt_state(13) << ","
               << gt_state(14) << "," << gt_state(15) << "," << gt_state(16) << std::endl;
    }
  }

  if (of_state.is_open()) {
    of_state.close();
  }

  err_ori.calculate();
  err_pos.calculate();
  err_vel.calculate();
  std::cout << "======================================" << std::endl;
  std::cout << "Trajectory Errors (deg,m,m/s)" << std::endl;
  std::cout << "======================================" << std::endl;
  std::cout << "rmse_ori = " << err_ori.rmse << " | rmse_pos = " << err_pos.rmse << " | rmse_vel = " << err_vel.rmse << std::endl;
  std::cout << "mean_ori = " << err_ori.mean << " | mean_pos = " << err_pos.mean << " | mean_vel = " << err_vel.mean << std::endl;
  std::cout << "min_ori  = " << err_ori.min << " | min_pos  = " << err_pos.min << " | min_vel  = " << err_vel.min << std::endl;
  std::cout << "max_ori  = " << err_ori.max << " | max_pos  = " << err_pos.max << " | max_vel  = " << err_vel.max << std::endl;
  std::cout << "std_ori  = " << err_ori.std << " | std_pos  = " << err_pos.std << " | std_vel  = " << err_vel.std << std::endl << std::endl;

  double toff;
  Eigen::Matrix3d R_BtoI, R_GtoV;
  Eigen::Vector3d p_BinI;
  solver.get_calibration(toff, R_BtoI, p_BinI, R_GtoV);
  Eigen::Vector4d q_BtoI = rot_2_quat(R_BtoI);
  Eigen::Vector4d gt_q_BtoI = rot_2_quat(sim->get_params().R_BtoI);

  std::cout << "======================================" << std::endl;
  std::cout << "Converged Calib vs Groundtruth" << std::endl;
  std::cout << "======================================" << std::endl;
  std::cout << "GT  toff: " << sim->get_params().viconimu_dt << std::endl;
  std::cout << "EST toff: " << toff << std::endl << std::endl;
  std::cout << "GT  rpy R_GtoV: " << rot2rpy(sim->get_params().R_GtoV)(0) << ", " << rot2rpy(sim->get_params().R_GtoV)(1) << ", "
            << rot2rpy(sim->get_params().R_GtoV)(2) << std::endl;
  std::cout << "EST rpy R_GtoV: " << rot2rpy(R_GtoV)(0) << ", " << rot2rpy(R_GtoV)(1) << ", " << rot2rpy(R_GtoV)(2) << std::endl
            << std::endl;
  std::cout << "GT  q_BtoI: " << gt_q_BtoI(0) << ", " << gt_q_BtoI(1) << ", " << gt_q_BtoI(2) << ", " << gt_q_BtoI(3) << std::endl;
  std::cout << "EST q_BtoI: " << q_BtoI(0) << ", " << q_BtoI(1) << ", " << q_BtoI(2) << ", " << q_BtoI(3) << std::endl << std::endl;
  std::cout << "GT  p_BinI: " << sim->get_params().p_BinI(0) << ", " << sim->get_params().p_BinI(1) << ", " << sim->get_params().p_BinI(2)
            << std::endl;
  std::cout << "EST p_BinI: " << p_BinI(0) << ", " << p_BinI(1) << ", " << p_BinI(2) << std::endl << std::endl;

  return EXIT_SUCCESS;
}