// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include "MotionGenerator.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>

void setDefaultBehavior(franka::Robot& robot) {
  robot.setCollisionBehavior(
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
  robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
  robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
}

// This is for gcc 5.4, does not need this line with gcc6.4
// Refer to https://stackoverflow.com/questions/8016780/undefined-reference-to-static-constexpr-char
constexpr std::array<double, 2> MotionGenerator::kDeltaMotionFinished;

MotionGenerator::MotionGenerator(double speed_factor, const std::array<double, 16> pose_goal)
    : pose_goal_(pose_goal.data()) {
  dq_max_ *= speed_factor;
  ddq_max_start_ *= speed_factor;
  ddq_max_goal_ *= speed_factor;
  dq_max_sync_.setZero();
  pose_start_.setZero();
  unit_T_.setZero();
  axis_R_.setZero();
  delta_q_.setZero();
  t_1_sync_.setZero();
  t_2_sync_.setZero();
  t_f_sync_.setZero();
  q_1_.setZero();
}

bool MotionGenerator::calculateDesiredValues(double t, Vector2d* delta_q_d) const {
  Vector2d t_d = t_2_sync_ - t_1_sync_;
  Vector2d delta_t_2_sync = t_f_sync_ - t_2_sync_;
  std::array<bool, 2> motion_finished{};

  for (size_t i = 0; i < 2; i++) {
    if (std::abs(delta_q_[i]) < kDeltaMotionFinished[i]) {
      (*delta_q_d)[i] = 0;
      motion_finished[i] = true;
    } else {
      if (t < t_1_sync_[i]) {
        (*delta_q_d)[i] = -1.0 / std::pow(t_1_sync_[i], 3.0) * dq_max_sync_[i] *
                          (0.5 * t - t_1_sync_[i]) * std::pow(t, 3.0);
      } else if (t >= t_1_sync_[i] && t < t_2_sync_[i]) {
        (*delta_q_d)[i] = q_1_[i] + (t - t_1_sync_[i]) * dq_max_sync_[i];
      } else if (t >= t_2_sync_[i] && t < t_f_sync_[i]) {
        (*delta_q_d)[i] =
            delta_q_[i] + 0.5 * (1.0 / std::pow(delta_t_2_sync[i], 3.0) *
                                   (t - t_2_sync_[i] - 2.0 * delta_t_2_sync[i]) *
                                   std::pow((t - t_2_sync_[i]), 3.0) +
                                (2.0 * t - 2.0 * t_2_sync_[i] - delta_t_2_sync[i])) * dq_max_sync_[i];
      } else {
        (*delta_q_d)[i] = delta_q_[i];
        motion_finished[i] = true;
      }
    }
  }
  return std::all_of(motion_finished.cbegin(), motion_finished.cend(),
                     [](bool x) { return x; });
}

void MotionGenerator::calculateSynchronizedValues() {
  Vector2d dq_max_reach(dq_max_);
  Vector2d t_f = Vector2d::Zero();
  Vector2d delta_t_2 = Vector2d::Zero();
  Vector2d t_1 = Vector2d::Zero();
  Vector2d delta_t_2_sync = Vector2d::Zero();

  unit_T_ = pose_goal_.block<3,1>(0,3) - pose_start_.block<3,1>(0,3);
  delta_q_[0] = unit_T_.norm();
  if (delta_q_[0] > 0) unit_T_ = unit_T_ / delta_q_[0];

  Matrix3x3d delta_R_;
  delta_R_ = pose_goal_.block<3,3>(0,0) * pose_start_.block<3,3>(0,0).transpose();
  // matrix to axis-angle
  double cos_angle = (delta_R_.trace()-1)/2;
  delta_q_[1] = std::acos(cos_angle);
  if (cos_angle < 0) {
    // more stable way of calculating axis when angle close to pi.
    axis_R_(0) = std::sqrt((delta_R_(0,0)-cos_angle)/2);
    axis_R_(1) = std::sqrt((delta_R_(1,1)-cos_angle)/2);
    axis_R_(2) = std::sqrt((delta_R_(2,2)-cos_angle)/2);
    if (delta_R_(0,1)-delta_R_(1,0) > 0) axis_R_(2) = -axis_R_(2);
    if (delta_R_(2,0)-delta_R_(0,2) > 0) axis_R_(1) = -axis_R_(1);
    if (delta_R_(1,2)-delta_R_(2,1) > 0) axis_R_(0) = -axis_R_(0);
    axis_R_.normalize();
  } else {
    // more stable way of calculating axis when angle close to 0.
    axis_R_(0) = delta_R_(2,1) - delta_R_(1,2);
    axis_R_(1) = delta_R_(0,2) - delta_R_(2,0);
    axis_R_(2) = delta_R_(1,0) - delta_R_(0,1);
    if (axis_R_.norm() > 0) axis_R_.normalize();
  }

  for (size_t i = 0; i < 2; i++) {
    if (std::abs(delta_q_[i]) > kDeltaMotionFinished[i]) {
      if (std::abs(delta_q_[i]) < (3.0 / 4.0 * (std::pow(dq_max_[i], 2.0) / ddq_max_start_[i]) +
                                   3.0 / 4.0 * (std::pow(dq_max_[i], 2.0) / ddq_max_goal_[i]))) {
        dq_max_reach[i] = std::sqrt(4.0 / 3.0 * delta_q_[i] *
                                    (ddq_max_start_[i] * ddq_max_goal_[i]) /
                                    (ddq_max_start_[i] + ddq_max_goal_[i]));
      }
      t_1[i] = 1.5 * dq_max_reach[i] / ddq_max_start_[i];
      delta_t_2[i] = 1.5 * dq_max_reach[i] / ddq_max_goal_[i];
      t_f[i] = t_1[i] / 2.0 + delta_t_2[i] / 2.0 + std::abs(delta_q_[i]) / dq_max_reach[i];
    }
  }
  double max_t_f = t_f.maxCoeff();
  for (size_t i = 0; i < 2; i++) {
    if (std::abs(delta_q_[i]) > kDeltaMotionFinished[i]) {
      double a = 1.5 / 2.0 * (ddq_max_goal_[i] + ddq_max_start_[i]);
      double b = -1.0 * max_t_f * ddq_max_goal_[i] * ddq_max_start_[i];
      double c = std::abs(delta_q_[i]) * ddq_max_goal_[i] * ddq_max_start_[i];
      double delta = b * b - 4.0 * a * c;
      if (delta < 0.0) {
        delta = 0.0;
      }
      dq_max_sync_[i] = (-1.0 * b - std::sqrt(delta)) / (2.0 * a);
      t_1_sync_[i] = 1.5 * dq_max_sync_[i] / ddq_max_start_[i];
      delta_t_2_sync[i] = 1.5 * dq_max_sync_[i] / ddq_max_goal_[i];
      t_f_sync_[i] =
          (t_1_sync_)[i] / 2.0 + delta_t_2_sync[i] / 2.0 + std::abs(delta_q_[i] / dq_max_sync_[i]);
      t_2_sync_[i] = t_f_sync_[i] - delta_t_2_sync[i];
      q_1_[i] = dq_max_sync_[i] * 0.5 * t_1_sync_[i];
    }
  }
}

void MotionGenerator::preCalculateRotationMatrices() {
  precompute_pose_eye_ = pose_start_.block<3,3>(0,0);
  precompute_pose_dot_ = axis_R_*axis_R_.transpose()*precompute_pose_eye_;
  precompute_pose_cross_ = Eigen::MatrixXd::Zero(3,3);
  precompute_pose_cross_(0,1) = -axis_R_(2);
  precompute_pose_cross_(1,0) = axis_R_(2);
  precompute_pose_cross_(0,2) = axis_R_(1);
  precompute_pose_cross_(2,0) = -axis_R_(1);
  precompute_pose_cross_(1,2) = -axis_R_(0);
  precompute_pose_cross_(2,1) = axis_R_(0);
  precompute_pose_cross_ = precompute_pose_cross_ * precompute_pose_eye_;
}

franka::CartesianPose MotionGenerator::operator()(const franka::RobotState& robot_state,
                                                   franka::Duration period) {
  time_ += period.toSec();

  if (time_ == 0.0) {
    pose_start_ = Matrix4x4d(robot_state.O_T_EE_c.data());
    calculateSynchronizedValues();
    preCalculateRotationMatrices();
  }

  Vector2d delta_q_d;
  bool motion_finished = calculateDesiredValues(time_, &delta_q_d);

  std::array<double, 16> command_pose;
  Eigen::Map<Matrix4x4d> command_pose_e(&command_pose[0]);
  command_pose_e = pose_start_;

  if (delta_q_d[0] > 0) {
    command_pose_e.block<3,1>(0,3) += delta_q_d[0] * unit_T_;
  }
  if (delta_q_d[1] > 0) {
    command_pose_e.block<3,3>(0,0) = std::cos(delta_q_d[1]) * precompute_pose_eye_ +
                                     (1 - std::cos(delta_q_d[1])) * precompute_pose_dot_ +
                                     std::sin(delta_q_d[1]) * precompute_pose_cross_;
  }
//  std::cout << command_pose_e << std::endl;
//  std::cout << command_pose_e.transpose()*command_pose_e << std::endl;
  franka::CartesianPose output(command_pose);
  output.motion_finished = motion_finished;
  return output;
}
