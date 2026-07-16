
import numpy as np
import matplotlib.pyplot as plt
from robot_util.arm_kinemarics import ArmKinematics

kin = ArmKinematics(urdf_path="/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/yahboomcar_description/urdf/rosmaster_m1pro.urdf.xacro")

pose = kin.forward_kinematics([90, 180, 0, 0, 90])
print(pose)
ik_result = kin.inverse_kinematics(0.31, 0.1, 0.17, 0, 1.57, 0)
print(ik_result)
