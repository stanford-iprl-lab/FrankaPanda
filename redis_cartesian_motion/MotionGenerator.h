#pragma once

#include <array>
#include <Eigen/Core>

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

/**
 * Sets a default collision behavior, joint impedance, Cartesian impedance, and filter frequency.
 *
 * @param[in] robot Robot instance to set behavior on.
 */
void setDefaultBehavior(franka::Robot& robot);

/**
 * Generate a joint pose motion to a goal position. Adapted from:
 * Wisama Khalil and Etienne Dombre. 2002. Modeling, Identification and Control of Robots
 * (Kogan Page Science Paper edition).
 */
class MotionGenerator {
 public:
  /**
   * Creates a new MotionGenerator instance for a target q.
   *
   * @param[in] speed_factor General speed factor in range [0, 1].
   * @param[in] pose_goal Target end-effector pose.
   */
  MotionGenerator(double speed_factor, const std::array<double, 16> pose_goal);

  /**
   * Sends next pose command
   *
   * @param[in] robot_state Current state of the robot.
   * @param[in] period Duration of execution.
   *
   * @return end-effector pose for use inside a control loop.
   */
  franka::CartesianPose operator()(const franka::RobotState& robot_state, franka::Duration period);

 private:
  using Vector3d = Eigen::Matrix<double, 3, 1, Eigen::ColMajor>;
  using Vector2d = Eigen::Matrix<double, 2, 1, Eigen::ColMajor>;
  using Matrix4x4d = Eigen::Matrix<double, 4, 4, Eigen::ColMajor>;
  using Matrix3x3d = Eigen::Matrix<double, 3, 3, Eigen::ColMajor>;

  bool calculateDesiredValues(double t, Vector2d* delta_pose_d) const;
  void calculateSynchronizedValues();
  void preCalculateRotationMatrices();

  // tranlational and rotation angle residual threshold
  static constexpr std::array<double, 2> kDeltaMotionFinished = {1e-4, 0.01};
  const Matrix4x4d pose_goal_;

  Matrix4x4d pose_start_;
  Vector3d unit_T_;
  Vector3d axis_R_;
  Vector2d delta_q_; // delta_trans_, delta_angle_

  Matrix3x3d precompute_pose_eye_;
  Matrix3x3d precompute_pose_dot_;
  Matrix3x3d precompute_pose_cross_;

  Vector2d dq_max_sync_;
  Vector2d t_1_sync_;
  Vector2d t_2_sync_;
  Vector2d t_f_sync_;
  Vector2d q_1_;

  double time_ = 0.0;

  Vector2d dq_max_ = (Vector2d() << 0.1, 0.4).finished(); // 0.2m/s, 1.0rad/s
  Vector2d ddq_max_start_ = (Vector2d() << 0.2, 0.5).finished();
  Vector2d ddq_max_goal_ = (Vector2d() << 0.2, 0.5).finished();
};
