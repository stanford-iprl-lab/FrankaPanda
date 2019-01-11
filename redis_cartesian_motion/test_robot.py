from robot import Robot

robot=Robot()
robot.absolute_command(robot.START_POSE)
robot.delta_command(0,0,-0.3,0)
robot.delta_command(0.1,0,0,0)
robot.delta_command(0,0.1,0,0)
robot.gripper_close(0.02)
robot.delta_command(-0.1,0,0,0)
robot.delta_command(0,-0.1,0,0)
robot.gripper_open()
print("Robot test passed")
