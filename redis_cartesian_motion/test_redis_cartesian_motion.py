import redis
import time
from datetime import datetime
import json
import numpy as np
#import pdb

EE_POSE_COMMANDED_KEY = "sai2::FrankaPanda::actuators::ee"
EE_POSE_COMMANDED_TYPE_KEY = "sai2::FrankaPanda::actuators::eeType"
COMMAND_TIMESTAMP_KEY = "sai2::FrankaPanda::actuators::T"
COMMAND_FINISH_FLAG_KEY = "sai2::FrankaPanda::actuators::done"
EE_POSE_KEY = "sai2::FrankaPanda::sensors::ee"
JOINT_ANGLES_KEY  = "sai2::FrankaPanda::sensors::q"

r = redis.Redis(host='localhost', port=6379, db=0)

def delta_command(dx, dy, dz, dtheta):
  ee_cmd_pose = [np.cos(dtheta), np.sin(dtheta), 0, 0,
                 -np.sin(dtheta), np.cos(dtheta), 0, 0,
                 0, 0, 1, 0,
                 dx, dy, dz, 1]
  timestamp = datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S.%f')
  print(timestamp)
  pipe = r.pipeline()
  pipe.set(EE_POSE_COMMANDED_KEY, json.dumps(ee_cmd_pose))
  pipe.set(EE_POSE_COMMANDED_TYPE_KEY, "DELTA")
  pipe.set(COMMAND_TIMESTAMP_KEY, timestamp)
  pipe.set(COMMAND_FINISH_FLAG_KEY, "false")
  pipe.execute()
  while True:
    done = r.get(COMMAND_FINISH_FLAG_KEY)
    if done != b'true':
      time.sleep(0.05)
    else:
      break
  ee_pose = r.get(EE_POSE_KEY)
  print(ee_pose)
  return

def absolute_command(target_pose):
  ee_cmd_pose = target_pose
  timestamp = datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S.%f')
  print(timestamp)
  pipe = r.pipeline()
  pipe.set(EE_POSE_COMMANDED_KEY, json.dumps(ee_cmd_pose))
  pipe.set(EE_POSE_COMMANDED_TYPE_KEY, "ABSOLUTE")
  pipe.set(COMMAND_TIMESTAMP_KEY, timestamp)
  pipe.set(COMMAND_FINISH_FLAG_KEY, "false")
  pipe.execute()
  while True:
    done = r.get(COMMAND_FINISH_FLAG_KEY)
    if done != b'true':
      time.sleep(0.05)
    else:
      break
  ee_pose = r.get(EE_POSE_KEY)
  print(ee_pose)
  return


START_POSE = [1, 0, 0, 0,
              0, -1, 0, 0,
              0, 0, -1, 0,
              0.45, 0, 0.5,1]

if __name__=="__main__":
  absolute_command(START_POSE)
  delta_command(0,0,-0.3,0)
  delta_command(0.1,0,0,0)
  delta_command(0,0.1,0,0)
  delta_command(-0.1,0,0,0)
  delta_command(0,-0.1,0,0)

