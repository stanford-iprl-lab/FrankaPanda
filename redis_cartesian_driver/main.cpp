// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <array>
#include <cmath>
#include <functional>
#include <iostream>

#include <Eigen/Core>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include "RedisClient.h"

// redis keys
// - read:
const std::string EE_POSE_COMMANDED_KEY = "sai2::FrankaPanda::actuators::ee";
const std::string COMMAND_TIMESTAMP_KEY = "sai2::FrankaPanda::actuators::T";
// - write:
const std::string JOINT_ANGLES_KEY  = "sai2::FrankaPanda::sensors::q";
const std::string EE_POSE_KEY = "sai2::FrankaPanda::sensors::ee";
const std::string SENSORS_TIMESTAMP_KEY = "sai2::FrankaPanda::sensors::T";
unsigned long long counter = 0;

void debug_function(CDatabaseRedisClient &redis_client)
{
  std::array<double, 16> ee_pose_cmd_array{};
  std::array<double, 16> ee_pose_sensed_array{};
  std::array<double, 7> q_array{};
  std::array<double, 2> cmd_time{};
  std::array<double, 2> sensor_time{};
  for (int i=0; i<3; i++) {
    ee_pose_sensed_array[i*4+i]=1;
  }
  q_array[3] = 1.5;
  std::chrono::nanoseconds start;
  double duration;

  redis_client.getDoubleArray(EE_POSE_COMMANDED_KEY, ee_pose_cmd_array, 16);
  redis_client.getDoubleArray(COMMAND_TIMESTAMP_KEY, cmd_time, 2);

  //std::vector<string> key_names;
  //key_names.push_back(JOINT_ANGLES_KEY);
  //key_names.push_back(EE_POSE_KEY);

  // TODO: make it string?
  //std::vector<std::array<double, 7>> sensor_feedback;
  //sensor_feedback.push_back(q_array);
  //sensor_feedback.push_back(ee_pose_sensed_array);

  while(true)
  {
    start = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::high_resolution_clock::now().time_since_epoch());

    //sensor_feedback.clear();
    //sensor_feedback.push_back(q_array);
    //sensor_feedback.push_back(ee_pose_sensed_array);

    //redis_client.setCommandBatch(key_names, sensor_feedback, 2);
    redis_client.setDoubleArray(JOINT_ANGLES_KEY, q_array, 7);
    redis_client.setDoubleArray(EE_POSE_KEY, ee_pose_sensed_array, 16);
    redis_client.getDoubleArray(EE_POSE_COMMANDED_KEY, ee_pose_cmd_array, 16);
    std::array<double, 16> new_pose = ee_pose_sensed_array;

    for (int i=12; i<16; i++){
      new_pose[i] = new_pose[i] + ee_pose_cmd_array[i];
    }

    Eigen::Matrix<double, 4, 4> current_pose_e(ee_pose_sensed_array.data());
    Eigen::Matrix<double, 4, 4> pose_cmd_e(ee_pose_cmd_array.data());
    Eigen::Matrix<double, 4, 4> new_pose_e;
    new_pose_e = pose_cmd_e * current_pose_e;

    for (int i=0; i<3; i++)
      for (int j=0; j<3; j++) {
        new_pose[i*4+j] = new_pose_e(j,i);
    }
    ee_pose_sensed_array = new_pose;

    std::chrono::nanoseconds t_end = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::high_resolution_clock::now().time_since_epoch());

    duration = ( t_end.count() - start.count() ) / 1e9;
    if(duration > 0.0003)
    {
      std::cout << "duration : "<< duration <<'\n';
      std::cout << "counter : " << counter << std::endl << std::endl;
    }

    counter++;
  }
}

int main(int argc, char** argv) {

  if(argc < 2)
  {
    std::cout << "Please enter the robot ip as an argument" << std::endl;
    return -1;
  }
  std::string robot_ip = argv[1];

  // start redis client
  CDatabaseRedisClient redis_client;
  HiredisServerInfo info;
  info.hostname_ = "127.0.0.1";
  info.port_ = 6379;
  info.timeout_ = { 1, 500000 }; // 1.5 seconds
  redis_client = CDatabaseRedisClient();
  redis_client.serverIs(info);

  debug_function(redis_client);
  return 0;

  std::array<double, 16> ee_pose_cmd_array{};
  std::array<double, 16> ee_pose_sensed_array{};
  std::array<double, 7> q_array{};

  redis_client.setDoubleArray(EE_POSE_COMMANDED_KEY, ee_pose_cmd_array, 16);
  // Safety to detect if controller is already running : wait 50 milliseconds
  usleep(50000);
  redis_client.getDoubleArray(EE_POSE_COMMANDED_KEY, ee_pose_cmd_array, 16);
  for(int i=0; i<16; i++)
  {
    if(ee_pose_cmd_array[i] != 0)
    {
      std::cout << "Stop the controller before runnung the driver\n" << std::endl;
      return -1;
    }
  }

  // prepare batch command
  // std::vector<string> key_names;
  // key_names.push_back(JOINT_TORQUES_COMMANDED_KEY);
  // key_names.push_back(MASSMATRIX_KEY);
  // key_names.push_back(JOINT_ANGLES_KEY);
  // key_names.push_back(JOINT_VELOCITIES_KEY);
  // key_names.push_back(JOINT_TORQUES_SENSED_KEY);
  // key_names.push_back(ROBOT_GRAVITY_KEY);
  // key_names.push_back(CORIOLIS_KEY);

  // std::vector<std::array<double, 7>> sensor_feedback;
  // sensor_feedback.push_back(q_array);
  // sensor_feedback.push_back(dq_array);
  // sensor_feedback.push_back(tau_sensed_array);
  // sensor_feedback.push_back(gravity_vector);
  // sensor_feedback.push_back(coriolis);

  std::clock_t start;
  double duration;

  try {
    // connect to robot and gripper
    franka::Robot robot(robot_ip);
    // load the kinematics and dynamics model
    franka::Model model = robot.loadModel();

    // set collision behavior
    robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

    auto cartesian_pose_callback = [&](const franka::RobotState& robot_state,
                                    franka::Duration period) -> franka::CartesianPose
    {
      // start = std::clock();
      ee_pose_sensed_array = robot_state.O_T_EE;
      std::array<double, 16> new_pose = ee_pose_sensed_array;
      redis_client.setDoubleArray(EE_POSE_KEY, ee_pose_sensed_array, 16);
      redis_client.setDoubleArray(JOINT_ANGLES_KEY, robot_state.q, 7);
      redis_client.getDoubleArray(EE_POSE_COMMANDED_KEY, ee_pose_cmd_array, 16);

      // Conversion, treating ee_pose_cmd_array as delta translation and rotation.
      for (int i=12; i<16; i++){
        new_pose[i] = new_pose[i] + ee_pose_cmd_array[i];
      }
      Eigen::Matrix<double, 4, 4> current_pose_e(ee_pose_sensed_array.data());
      Eigen::Matrix<double, 4, 4> pose_cmd_e(ee_pose_cmd_array.data());
      Eigen::Matrix<double, 4, 4> new_pose_e;
      new_pose_e = pose_cmd_e * current_pose_e;
      for (int i=0; i<3; i++)
        for (int j=0; j<3; j++) {
          new_pose[i*4+j] = new_pose_e(j,i);
      }

      // sensor_feedback[0] = robot_state.q;
      // sensor_feedback[1] = robot_state.dq;
      // sensor_feedback[2] = robot_state.tau_J;
      // sensor_feedback[3] = model.gravity(robot_state);
      // sensor_feedback[4] = model.coriolis(robot_state);

      // M_array = model.mass(robot_state);
      // Eigen::Map<const Eigen::Matrix<double, 7, 7> > MassMatrix(M_array.data());
      // redis_client.setGetBatchCommands(key_names, tau_cmd_array, MassMatrix, sensor_feedback);

      // if(counter % 100 == 0)
      // {
      //   std::cout << "joint angles : ";
      //   for(int i=0; i<7; i++)
      //   {
      //     std::cout << " " << robot_state.q[i] << " ";
      //   }
      //   std::cout << std::endl;
      //   std::cout << "joint velocities : ";
      //   for(int i=0; i<7; i++)
      //   {
      //     std::cout << " " << robot_state.dq[i] << " ";
      //   }
      //   std::cout << std::endl;
      //   std::cout << "joint torques sensed : ";
      //   for(int i=0; i<7; i++)
      //   {
      //     std::cout << " " << robot_state.tau_J[i] << " ";
      //   }
      //   std::cout << std::endl;
      //   std::cout << "torque commanded : ";
      //   for(int i=0; i<7; i++)
      //   {
      //     std::cout << " " << tau_cmd_array[i] << " ";
      //   }
      //   std::cout << std::endl;
      //   std::cout << std::endl;
      // }

      // for(int i=0; i<7; i++)
      // {
      //   tau_cmd_array[i] = 0;
      // }

      counter++;
      return new_pose;
    };

    // start real-time control loop
    robot.control(cartesian_pose_callback);

  } catch (const std::exception& ex) {
    // print exception
    std::cout << ex.what() << std::endl;
    std::cout << "counter : " << counter << std::endl;
  }

  return 0;
}
