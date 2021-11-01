import pybullet_data as pd
import pybullet as p
import numpy as np
import time

p.connect(p.GUI)
p.setAdditionalSearchPath(pd.getDataPath())
# p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
ground_id = p.loadURDF('plane.urdf')
p.setGravity(0, 0, -9.8)
# anymal_b = p.loadURDF('model/anymal_b_simple_description-master/anymal.xml', [0, -0.5, 0.6])

# anymal_c = p.loadURDF('model/anymal_c_simple_description-master/anymal.xml', [0, 0.5, 0.8])

# dog = p.loadURDF('model/laikago/laikago_toes_zup.xml', [0, 0, 0.6])
p.loadURDF('../quadruped_model/bosdyn_spot/model.xml', [0, 0, 1.0], useFixedBase=True)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

body_size = [0.29785, 0.055, 0]
upper_leg_location = [0, 0.110945, 0]
lower_leg_location = [0.025, 0, -0.3205]
foot_location = [0, 0, -0.37]
# b_joint_num = p.getNumJoints(anymal_b)
# c_joint_num = p.getNumJoints(anymal_c)

# motor_list = []
# revolute_joint_name = []
# feet_id = [44, 54, 64, 74]
# for i in range(16):
#     info = p.getJointInfo(dog, i)
#     if info[2] == 0:
#         motor_list.append(info[0])
#         revolute_joint_name.append(info[1])
# print('-'*20)
# print(motor_list)


init_joint_pos = [-0.5, 0.8, -1.6,
                  0.5, 0.8, -1.6,
                  -0.5, 0.8, -1.6,
                  0.5, 0.8, -1.6]

# init_joint_pos2 = [0]*12
# init_joint_pos = [0.0]*12
# for i in range(12):
#     p.setJointMotorControl2(dog, motor_list[i], p.POSITION_CONTROL, init_joint_pos[i])

# for i in range(4):
#     p.changeDynamics(anymal_c, feet_id[i], spinningFriction=0.0065)

p.changeDynamics(ground_id, -1, lateralFriction=1)

while 1:
    p.stepSimulation()
    time.sleep(0.02)
