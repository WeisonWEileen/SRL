import time
import threading
from interbotix_xs_modules.arm import InterbotixManipulatorXS

class vx300Thread1(threading.Thread):
    def __init__(self):
        self.robot= InterbotixManipulatorXS(robot_model="vx300", group_name="arm", gripper_name="gripper", robot_name="vx300", moving_time=0.5, accel_time=0.2, gripper_pressure=0.5, gripper_pressure_lower_limit=150, gripper_pressure_upper_limit=350, init_node=False)

    def hit(self, position):
        if(position is not None):
            self.robot.arm.set_ee_pose_components(x=position.x, y=position.y, z=position.z, moving_time=1) #击打目标位置
            time.sleep(3)                                                                             #等待3s
            self.robot.arm.set_joint_positions([0.0, -0.4, 1.0, -0.6, 0.0], moving_time=1)                 #回到准备姿态

    def run(self):
        self.robot.arm.go_to_sleep_pose(moving_time=1)
        self.robot.arm.go_to_home_pose(moving_time=1)
        self.robot.arm.set_joint_positions([0.0, -0.4, 1.0, -0.6, 0.0], moving_time=1) 



class vx300Thread2(threading.Thread):
    def __init__(self):
        self.robot= InterbotixManipulatorXS(robot_model="vx300", group_name="arm", gripper_name="gripper", robot_name="vx300", moving_time=0.5, accel_time=0.2, gripper_pressure=0.5, gripper_pressure_lower_limit=150, gripper_pressure_upper_limit=350, init_node=False)

    def hit(self, position):
        if(position is not None):
            self.robot.arm.set_ee_pose_components(x=position.x, y=position.y, z=position.z, moving_time=1) #击打目标位置
            time.sleep(3)                                                                             #等待3s
            self.robot.arm.set_joint_positions([0.0, -0.4, 1.0, -0.6, 0.0], moving_time=1)                 #回到准备姿态

    def run(self):
        self.robot.arm.go_to_sleep_pose(moving_time=1)
        self.robot.arm.go_to_home_pose(moving_time=1)
        self.robot.arm.set_joint_positions([0.0, -0.4, 1.0, -0.6, 0.0], moving_time=1) 