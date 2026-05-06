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
#ifndef VICONGRAPHSOLVER_H
#define VICONGRAPHSOLVER_H

#include <Eigen/Eigen>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem.hpp>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>

#include "cpi/CpiV1.h"
#include "gtsam/GtsamConfig.h"
#include "gtsam/ImuFactorCPIv1.h"
#include "gtsam/JPLNavState.h"
#include "gtsam/JPLQuaternion.h"
#include "gtsam/MeasBased_ViconPoseTimeoffsetFactor.h"
#include "gtsam/RotationXY.h"
#include "meas/Interpolator.h"
#include "meas/Propagator.h"
#include "utils/quat_ops.h"

using namespace std;
using namespace gtsam;

using gtsam::symbol_shorthand::C;
using gtsam::symbol_shorthand::G;
using gtsam::symbol_shorthand::T;
using gtsam::symbol_shorthand::X;

class ViconGraphSolver {

public:
  ViconGraphSolver(const std::string &config_file, std::shared_ptr<Propagator> propagator, std::shared_ptr<Interpolator> interpolator,
                   std::vector<double> timestamp_cameras);

  void build_and_solve();

  void write_to_file(std::string csvfilepath, std::string infofilepath);

  void visualize();

  void get_imu_poses(std::vector<double> &times, std::vector<Eigen::Matrix<double, 10, 1>> &poses);

  void get_calibration(double &toff, Eigen::Matrix3d &R_BtoI, Eigen::Vector3d &p_BinI, Eigen::Matrix3d &R_GtoV);

protected:
  void build_problem(bool init_states);

  void optimize_problem();

  boost::posix_time::ptime rT1, rT2, rT3, rT4, rT5, rT6, rT7;

  std::shared_ptr<Propagator> propagator;
  std::shared_ptr<Interpolator> interpolator;
  std::vector<double> timestamp_cameras;

  Eigen::Matrix3d init_R_GtoV;
  Eigen::Matrix3d init_R_BtoI;
  Eigen::Vector3d init_p_BinI;
  double init_toff_imu_to_vicon;

  double gravity_magnitude;

  gtsam::NonlinearFactorGraph *graph;
  gtsam::Values values;

  gtsam::Values values_result;

  std::shared_ptr<GtsamConfig> config;

  std::map<double, size_t> map_states;

  int num_loop_relin;

  double TIME_OFFSET = 0.25;

  std::string config_file_;
};

#endif /* VICONGRAPHSOLVER_H */