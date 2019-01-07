// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include <chrono>
#include <thread>
#include <Eigen/Core>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include "RedisClient.h"
#include "MotionGenerator.h"

// redis keys
// - read:
const std::string EE_POSE_COMMANDED_KEY = "sai2::FrankaPanda::actuators::ee";
const std::string EE_POSE_COMMANDED_TYPE_KEY = "sai2::FrankaPanda::actuators::eeType";
const std::string COMMAND_TIMESTAMP_KEY = "sai2::FrankaPanda::actuators::T";
const std::string COMMAND_FINISH_FLAG_KEY = "sai2::FrankaPanda::actuators::done";
// - write:
const std::string JOINT_ANGLES_KEY  = "sai2::FrankaPanda::sensors::q";
const std::string EE_POSE_KEY = "sai2::FrankaPanda::sensors::ee";
const std::string SENSORS_TIMESTAMP_KEY = "sai2::FrankaPanda::sensors::T";
unsigned long long counter = 0;

std::string time_point_to_string(std::chrono::system_clock::time_point &tp)
{
  auto ttime_t = std::chrono::system_clock::to_time_t(tp);
  auto tp_sec = std::chrono::system_clock::from_time_t(ttime_t);
  std::chrono::microseconds ms = std::chrono::duration_cast<std::chrono::microseconds>(tp - tp_sec);

  std::tm * ttm = std::gmtime(&ttime_t);

  char date_time_format[] = "%Y-%m-%d %H:%M:%S";
  char time_str[] = "yyyy-mm-dd HH:MM:SS.ffffff";

  std::strftime(time_str, strlen(time_str), date_time_format, ttm);

  std::string result(time_str);
  result.append(".");
  result.append(std::to_string(ms.count()));

  return result;
}

void debug_function(RedisClient &redis_client)
{
  std::array<double, 16> ee_pose_cmd_array{};
  std::array<double, 16> ee_pose_sensed_array{};
  std::array<double, 7> q_array{};

  for (int i=0; i<4; i++) {
    ee_pose_sensed_array[i*4+i]=1;
  }

  std::chrono::nanoseconds start;
  double duration;

//  redis_client.getDoubleArray(EE_POSE_COMMANDED_KEY, ee_pose_cmd_array, 16);
//  redis_client.getDoubleArray(COMMAND_TIMESTAMP_KEY, cmd_time, 2);

  for (int i=0; i<4; i++) {
    ee_pose_cmd_array[i*4+i]=1;
  }

//  ee_pose_cmd_array[0]=0;
//  ee_pose_cmd_array[1]=1;
//  ee_pose_cmd_array[4]=-1;
//  ee_pose_cmd_array[5]=0;

  ee_pose_cmd_array[12] = 1;

  franka::RobotState robot_state;
  robot_state.O_T_EE_c = ee_pose_sensed_array;
  franka::Duration franka_duration(1);
  franka::CartesianPose output(ee_pose_sensed_array);
  output.motion_finished = false;
  MotionGenerator motion_generator(1.0, ee_pose_cmd_array);
  output = motion_generator(robot_state, franka::Duration(0));

  while(!output.motion_finished)
  {
    start = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::high_resolution_clock::now().time_since_epoch());

    output = motion_generator(robot_state, franka_duration);

    std::chrono::nanoseconds t_end = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::high_resolution_clock::now().time_since_epoch());
    for (int i=0; i<16; i++) {
      std::cout << output.O_T_EE[i] << "  ";
    }
    std::cout << std::endl;

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
  RedisClient redis_client;
  redis_client.connect("localhost", 6379);

//  debug_function(redis_client);
//  return 0;

  std::array<double, 16> ee_pose_cmd_array{};
  std::array<double, 16> ee_pose_sensed_array{};
  std::array<double, 7> q_array{};
  std::string ee_pose_cmd_array_str;
  std::string ee_pose_sensed_array_str;
  std::string previous_command_time;
  std::string current_command_time;
  std::string command_type;
  std::vector<std::string> request_keys;
  std::vector<std::string> replies;
  request_keys.push_back(COMMAND_TIMESTAMP_KEY);
  request_keys.push_back(EE_POSE_COMMANDED_KEY);
  request_keys.push_back(EE_POSE_COMMANDED_TYPE_KEY);

  auto timestamp = std::chrono::system_clock::now();
  std::string timestamp_s = time_point_to_string(timestamp);

  redis_client.hDoubleArraytoStringJSON(ee_pose_cmd_array, 16, ee_pose_cmd_array_str);
  redis_client.set(EE_POSE_COMMANDED_KEY, ee_pose_cmd_array_str);
  redis_client.set(COMMAND_TIMESTAMP_KEY, timestamp_s);
  // Safety to detect if controller is already running : wait 50 milliseconds
  usleep(50000);
  ee_pose_cmd_array_str = redis_client.get(EE_POSE_COMMANDED_KEY);
  previous_command_time = redis_client.get(COMMAND_TIMESTAMP_KEY);
  redis_client.hDoubleArrayfromStringJSON(ee_pose_cmd_array, 16, ee_pose_cmd_array_str);

  for(int i=0; i<16; i++)
  {
    if(ee_pose_cmd_array[i] != 0)
    {
      std::cout << "Stop the controller before runnung the driver\n" << std::endl;
      return -1;
    }
  }
  if (previous_command_time != timestamp_s) {
    std::cout << "Stop the controller before runnung the driver\n" << std::endl;
    return -1;
  }

  // For debug only
  for (int i=0; i<4; i++) {
    ee_pose_sensed_array[i*4+i]=1;
  }
  for (int i=0; i<4; i++) {
    ee_pose_cmd_array[i*4+i]=1;
  }

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


    franka::RobotState robot_state;

    while (true) {
      replies = redis_client.mget(request_keys);
      current_command_time = replies[0];
      redis_client.hDoubleArrayfromStringJSON(ee_pose_cmd_array, 16, replies[1]);
      command_type = replies[2];
      if (current_command_time == previous_command_time) std::this_thread::sleep_for(std::chrono::milliseconds(5));
      else {
        previous_command_time = current_command_time;
        if (command_type == "DELTA") {
          robot_state = robot.readOnce();
          ee_pose_sensed_array = robot_state.O_T_EE_c;

          Eigen::Map<Eigen::Matrix4d> start_pose_e(&ee_pose_sensed_array[0]);
          Eigen::Map<Eigen::Matrix4d> command_pose_e(&ee_pose_cmd_array[0]);
          command_pose_e.block<3,1>(0,3) += start_pose_e.block<3,1>(0,3);
          command_pose_e.block<3,3>(0,0) = command_pose_e.block<3,3>(0,0) * start_pose_e.block<3,3>(0,0);
        }

        for (int i=0; i<16; i++) std::cout << ee_pose_sensed_array[i] << "  ";
        std::cout << std::endl;
        for (int i=0; i<16; i++) std::cout << ee_pose_cmd_array[i] << "  ";
        std::cout << std::endl;

        MotionGenerator motion_generator(1.0, ee_pose_cmd_array);
        robot.control(motion_generator, franka::ControllerMode::kCartesianImpedance);
        std::cout << "Finished moving to desired pose" << std::endl;
        robot_state = robot.readOnce();
        ee_pose_sensed_array = robot_state.O_T_EE_c;
        //ee_pose_sensed_array = ee_pose_cmd_array;
        redis_client.hDoubleArraytoStringJSON(ee_pose_sensed_array, 16, ee_pose_sensed_array_str);
        redis_client.set(EE_POSE_KEY, ee_pose_sensed_array_str);
        redis_client.set(COMMAND_FINISH_FLAG_KEY, "true");
      }
    }

  } catch (const std::exception& ex) {
    // print exception
    std::cout << ex.what() << std::endl;
    std::cout << "counter : " << counter << std::endl;
  }

  return 0;
}
