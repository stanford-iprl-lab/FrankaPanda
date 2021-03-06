// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <array>
#include <string>
#include <cmath>
#include <functional>
#include <iostream>

#include <Eigen/Core>

#include <franka/exception.h>
#include <franka/gripper.h>

#include "RedisClient.h"

// - gripper
const std::string GRIPPER_MODE_KEY  = "sai2::FrankaPanda::gripper::mode"; // m for move and g for graps
const std::string GRIPPER_MAX_WIDTH_KEY  = "sai2::FrankaPanda::gripper::max_width";
const std::string GRIPPER_CURRENT_WIDTH_KEY  = "sai2::FrankaPanda::gripper::current_width";
const std::string GRIPPER_DESIRED_WIDTH_KEY  = "sai2::FrankaPanda::gripper::desired_width";
const std::string GRIPPER_DESIRED_SPEED_KEY  = "sai2::FrankaPanda::gripper::desired_speed";
const std::string GRIPPER_DESIRED_FORCE_KEY  = "sai2::FrankaPanda::gripper::desired_force";
const std::string GRIPPER_COMMAND_FINISH_FLAG_KEY  = "sai2::FrankaPanda::gripper::command_finished";

#include <signal.h>
bool runloop = false;
void sighandler(int){runloop = false;}
unsigned long long counter = 0;


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

  // set up signal handler
  signal(SIGABRT, &sighandler);
  signal(SIGTERM, &sighandler);
  signal(SIGINT, &sighandler);

  // setup variables for gripper control
  double gripper_desired_width, gripper_desired_width_tmp;
  double gripper_desired_speed, gripper_desired_speed_tmp;
  double gripper_desired_force, gripper_desired_force_tmp;
  double gripper_max_width;
  std::string gripper_mode = "m";
  std::string gripper_mode_tmp;

  bool flag_command_changed = false;

  try {
    // connect to gripper
    franka::Gripper gripper(robot_ip);

    // home the gripper
    // gripper.homing();

    franka::GripperState gripper_state = gripper.readOnce();
    gripper_max_width = gripper_state.max_width;
    redis_client.set(GRIPPER_MAX_WIDTH_KEY, std::to_string(gripper_max_width));
    redis_client.set(GRIPPER_MODE_KEY, gripper_mode);

    // gripper_desired_width = 0.5*gripper_max_width;
    gripper_desired_width = gripper_state.width;
    gripper_desired_speed = 0.07;
    gripper_desired_force = 0.0;
    gripper.move(gripper_desired_width, gripper_desired_speed);

    redis_client.set(GRIPPER_DESIRED_WIDTH_KEY, std::to_string(gripper_desired_width));
    redis_client.set(GRIPPER_DESIRED_SPEED_KEY, std::to_string(gripper_desired_speed));
    redis_client.set(GRIPPER_DESIRED_FORCE_KEY, std::to_string(gripper_desired_force));

    runloop = true;
    while(runloop)
    {
      gripper_state = gripper.readOnce();
      redis_client.set(GRIPPER_CURRENT_WIDTH_KEY, std::to_string(gripper_state.width));

      std::vector<std::string> get_keys;
      get_keys.push_back(GRIPPER_DESIRED_WIDTH_KEY);
      get_keys.push_back(GRIPPER_DESIRED_SPEED_KEY);
      get_keys.push_back(GRIPPER_DESIRED_FORCE_KEY);
      get_keys.push_back(GRIPPER_MODE_KEY);
      std::vector<std::string> replies = redis_client.mget(get_keys);
      gripper_desired_width_tmp = atof(replies[0].c_str());
      gripper_desired_speed_tmp = atof(replies[1].c_str());
      gripper_desired_force_tmp = atof(replies[2].c_str());
      gripper_mode_tmp = replies[3];

      if(gripper_desired_width_tmp > gripper_max_width)
      {
        gripper_desired_width_tmp = gripper_max_width;
        redis_client.set(GRIPPER_DESIRED_WIDTH_KEY, std::to_string(gripper_max_width));
        std::cout << "WARNING : Desired gripper width higher than max width. saturating to max width\n" << std::endl;
      }
      if(gripper_desired_width_tmp < 0)
      {
        gripper_desired_width_tmp = 0;
        redis_client.set(GRIPPER_DESIRED_WIDTH_KEY, std::to_string(0));
        std::cout << "WARNING : Desired gripper width lower than 0. saturating to max 0\n" << std::endl;
      }
      if(gripper_desired_speed_tmp < 0)
      {
        gripper_desired_speed_tmp = 0;
        redis_client.set(GRIPPER_DESIRED_SPEED_KEY, std::to_string(0));
        std::cout << "WARNING : Desired gripper speed lower than 0. saturating to max 0\n" << std::endl;
      }
      if(gripper_desired_force_tmp < 0)
      {
        gripper_desired_force_tmp = 0;
        redis_client.set(GRIPPER_DESIRED_FORCE_KEY, std::to_string(0));
        std::cout << "WARNING : Desired gripper speed lower than 0. saturating to max 0\n" << std::endl;
      }

      if(gripper_desired_width != gripper_desired_width_tmp ||
         gripper_desired_speed != gripper_desired_speed_tmp ||
         gripper_desired_force != gripper_desired_force_tmp ||
         gripper_mode != gripper_mode_tmp)
      {
        flag_command_changed = true;
        gripper_desired_width = gripper_desired_width_tmp;
        gripper_desired_speed = gripper_desired_speed_tmp;
        gripper_desired_force = gripper_desired_force_tmp;
        gripper_mode = gripper_mode_tmp;
      }

      if(flag_command_changed)
      {
        redis_client.set(GRIPPER_COMMAND_FINISH_FLAG_KEY, "false");
        if(gripper_mode == "m")
        {
          std::cout << "moving" << std::endl;
          gripper.move(gripper_desired_width, gripper_desired_speed);
        }
        else if(gripper_mode == "g")
        {
          std::cout << "grasping" << gripper_desired_width << ":" << gripper_desired_speed << ":" << gripper_desired_force << std::endl;
          gripper.grasp(gripper_desired_width, gripper_desired_speed, gripper_desired_force,
                        0.05, 0.05);
        }
        else
        {
          std::cout << "WARNING : gripper mode not recognized. Use g for grasp and m for move\n"<< std::endl;
        }
        flag_command_changed = false;
        redis_client.set(GRIPPER_COMMAND_FINISH_FLAG_KEY, "true");
      }

      // if(counter % 500 == 0)
      // {
        // std::cout << "counter : " << counter << std::endl;
      // }
      counter++;
    }

    gripper.stop();


  } catch (const std::exception& ex) {
    // print exception
    std::cout << ex.what() << std::endl;
  }



  return 0;
}
