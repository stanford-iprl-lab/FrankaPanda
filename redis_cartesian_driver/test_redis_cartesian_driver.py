import redis
import time
import json
import numpy as np

EE_POSE_COMMANDED_KEY = "sai2::FrankaPanda::actuators::ee";
EE_POSE_KEY = "sai2::FrankaPanda::sensors::ee";
JOINT_ANGLES_KEY  = "sai2::FrankaPanda::sensors::q";

r = redis.Redis(host='localhost', port=6379, db=0)
ee_cmd_pose = [np.cos(0.01), np.sin(0.01), 0, 0,
               -np.sin(0.01), np.cos(0.01), 0, 0,
               0, 0, 1, 0,
               0.02, -0.02, 0.03, 0]
while True:
    joint_angle = r.get(JOINT_ANGLES_KEY)
    ee_pose = r.get(EE_POSE_KEY)
    print(ee_pose)
    r.set(EE_POSE_COMMANDED_KEY, json.dumps(ee_cmd_pose))
    time.sleep(0.001)

