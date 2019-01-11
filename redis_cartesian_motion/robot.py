import redis
import time
from datetime import datetime
import json
import numpy as np
#import pdb

class Robot(object):
  def __init__(self, host='localhost'):
    self.EE_POSE_COMMANDED_KEY = "sai2::FrankaPanda::actuators::ee"
    self.EE_POSE_COMMANDED_TYPE_KEY = "sai2::FrankaPanda::actuators::eeType"
    self.COMMAND_TIMESTAMP_KEY = "sai2::FrankaPanda::actuators::T"
    self.COMMAND_FINISH_FLAG_KEY = "sai2::FrankaPanda::actuators::done"
    self.EE_POSE_KEY = "sai2::FrankaPanda::sensors::ee"
    self.JOINT_ANGLES_KEY  = "sai2::FrankaPanda::sensors::q"

    self.GRIPPER_MODE_KEY  = "sai2::FrankaPanda::gripper::mode" # m for move and g for grasp
    self.GRIPPER_MAX_WIDTH_KEY  = "sai2::FrankaPanda::gripper::max_width"
    self.GRIPPER_CURRENT_WIDTH_KEY  = "sai2::FrankaPanda::gripper::current_width"
    self.GRIPPER_DESIRED_WIDTH_KEY  = "sai2::FrankaPanda::gripper::desired_width"
    self.GRIPPER_DESIRED_SPEED_KEY  = "sai2::FrankaPanda::gripper::desired_speed"
    self.GRIPPER_DESIRED_FORCE_KEY  = "sai2::FrankaPanda::gripper::desired_force"
    self.GRIPPER_COMMAND_FINISH_FLAG_KEY = "sai2::FrankaPanda::gripper::command_finished"

    self.START_POSE = [1, 0, 0, 0,
                       0, -1, 0, 0,
                       0, 0, -1, 0,
                       0.45, 0, 0.5,1]

    self.redis_client = redis.Redis(host='localhost', port=6379, db=0)


  def delta_command(self, dx, dy, dz, dtheta):
    ee_cmd_pose = [np.cos(dtheta), np.sin(dtheta), 0, 0,
                   -np.sin(dtheta), np.cos(dtheta), 0, 0,
                   0, 0, 1, 0,
                   dx, dy, dz, 1]
    timestamp = datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S.%f')
    print(timestamp)
    pipe = self.redis_client.pipeline()
    pipe.set(self.EE_POSE_COMMANDED_KEY, json.dumps(ee_cmd_pose))
    pipe.set(self.EE_POSE_COMMANDED_TYPE_KEY, "DELTA")
    pipe.set(self.COMMAND_TIMESTAMP_KEY, timestamp)
    pipe.set(self.COMMAND_FINISH_FLAG_KEY, "false")
    pipe.execute()
    while True:
      done = self.redis_client.get(self.COMMAND_FINISH_FLAG_KEY)
      if done != b'true':
        time.sleep(0.05)
      else:
        break
    ee_pose = self.redis_client.get(self.EE_POSE_KEY)
    print(ee_pose)
    return

  def absolute_command(self, target_pose):
    ee_cmd_pose = target_pose
    timestamp = datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S.%f')
    print(timestamp)
    pipe = self.redis_client.pipeline()
    pipe.set(self.EE_POSE_COMMANDED_KEY, json.dumps(ee_cmd_pose))
    pipe.set(self.EE_POSE_COMMANDED_TYPE_KEY, "ABSOLUTE")
    pipe.set(self.COMMAND_TIMESTAMP_KEY, timestamp)
    pipe.set(self.COMMAND_FINISH_FLAG_KEY, "false")
    pipe.execute()
    while True:
      done = self.redis_client.get(self.COMMAND_FINISH_FLAG_KEY)
      if done != b'true':
        time.sleep(0.05)
      else:
        break
    ee_pose = self.redis_client.get(self.EE_POSE_KEY)
    print(ee_pose)
    return

  def gripper_close(self, width, speed=0.05, force=10.0):
    # Gripper will try to open and reclose if desired width is too small.
    # Thus width should be set close to the object width.
    pipe = self.redis_client.pipeline()
    pipe.set(self.GRIPPER_COMMAND_FINISH_FLAG_KEY, b'false')
    pipe.set(self.GRIPPER_MODE_KEY, "g")
    pipe.set(self.GRIPPER_DESIRED_WIDTH_KEY, str(width))
    pipe.set(self.GRIPPER_DESIRED_SPEED_KEY, str(speed))
    pipe.set(self.GRIPPER_DESIRED_FORCE_KEY, str(force))
    pipe.execute()
    while True:
      done = self.redis_client.get(self.GRIPPER_COMMAND_FINISH_FLAG_KEY)
      if done != b'true':
        time.sleep(0.05)
      else:
        break
    return

  def gripper_open(self, width=0.05, speed=0.05):
    pipe = self.redis_client.pipeline()
    pipe.set(self.GRIPPER_COMMAND_FINISH_FLAG_KEY, b'false')
    pipe.set(self.GRIPPER_MODE_KEY, "m")
    pipe.set(self.GRIPPER_DESIRED_WIDTH_KEY, str(width))
    pipe.set(self.GRIPPER_DESIRED_SPEED_KEY, str(speed))
    pipe.set(self.GRIPPER_DESIRED_FORCE_KEY, str(0.0))
    pipe.execute()
    while True:
      done = self.redis_client.get(self.GRIPPER_COMMAND_FINISH_FLAG_KEY)
      if done != b'true':
        time.sleep(0.05)
      else:
        break
    return


