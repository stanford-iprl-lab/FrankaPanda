import redis
import time
from datetime import datetime
import json
import numpy as np

GRIPPER_MODE_KEY  = "sai2::FrankaPanda::gripper::mode" # m for move and g for grasp
GRIPPER_MAX_WIDTH_KEY  = "sai2::FrankaPanda::gripper::max_width"
GRIPPER_CURRENT_WIDTH_KEY  = "sai2::FrankaPanda::gripper::current_width"
GRIPPER_DESIRED_WIDTH_KEY  = "sai2::FrankaPanda::gripper::desired_width"
GRIPPER_DESIRED_SPEED_KEY  = "sai2::FrankaPanda::gripper::desired_speed"
GRIPPER_DESIRED_FORCE_KEY  = "sai2::FrankaPanda::gripper::desired_force"
GRIPPER_COMMAND_FINISH_FLAG_KEY = "sai2::FrankaPanda::gripper::command_finished"

r = redis.Redis(host='localhost', port=6379, db=0)

def gripper_close(width, speed, force):
  # Gripper will try to open and reclose if desired width is too small.
  # Thus width should be set close to the object width.
  pipe = r.pipeline()
  pipe.set(GRIPPER_COMMAND_FINISH_FLAG_KEY, b'false')
  pipe.set(GRIPPER_MODE_KEY, "g")
  pipe.set(GRIPPER_DESIRED_WIDTH_KEY, str(width))
  pipe.set(GRIPPER_DESIRED_SPEED_KEY, str(speed))
  pipe.set(GRIPPER_DESIRED_FORCE_KEY, str(force))
  pipe.execute()
  while True:
    done = r.get(GRIPPER_COMMAND_FINISH_FLAG_KEY)
    if done != b'true':
      time.sleep(0.05)
    else:
      break
  return

def gripper_open(width, speed):
  pipe = r.pipeline()
  pipe.set(GRIPPER_COMMAND_FINISH_FLAG_KEY, b'false')
  pipe.set(GRIPPER_MODE_KEY, "m")
  pipe.set(GRIPPER_DESIRED_WIDTH_KEY, str(width))
  pipe.set(GRIPPER_DESIRED_SPEED_KEY, str(speed))
  pipe.set(GRIPPER_DESIRED_FORCE_KEY, str(0.0))
  pipe.execute()
  while True:
    done = r.get(GRIPPER_COMMAND_FINISH_FLAG_KEY)
    if done != b'true':
      time.sleep(0.05)
    else:
      break
  return

if __name__=="__main__":
  gripper_close(0.02, 0.05, 10.0)
  print("gripper move finished")
  time.sleep(5.0)
  gripper_open(0.05, 0.05)
  print("gripper move finished")

