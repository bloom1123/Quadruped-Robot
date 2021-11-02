
import os
import numpy as np
import pyquaternion
import matplotlib.pyplot as plt
import time
import random
import ctypes
from PIL import Image as pil
import pybullet as p
import pybullet_data
from pybullet_utils import gazebo_world_parser
from build.robot_controller import robot_controller
from JoystickInterface import JoystickInterface

get_last_vel = [0] * 3

spot_motor_id_list = [0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14]
anymal_motor_id_list = [0, 1, 2, 5, 6, 7, 10, 11, 12, 15, 16, 17]

spot_foot_id = [3, 7, 11, 15]
anymal_foot_id = [4, 9, 14, 19]

spot_init_new_pos = [-0.1, 0.8, -1.6,
                0.1, 0.8, -1.6,
                -0.1, 0.8, -1.6, 
                0.1, 0.8, -1.6]

anymal_init_new_pos = [0.1, 0.8, -1.6,
                -0.1, 0.8, -1.6,
                0.1, -0.8, 1.6,
                -0.1, -0.8, 1.6]

quadruped_name = "spot"

if quadruped_name == "spot":
    init_new_pos = spot_init_new_pos
    motor_id_list = spot_motor_id_list
    foot_id = spot_foot_id

elif quadruped_name == "anymal_b":
    init_new_pos = anymal_init_new_pos
    motor_id_list = anymal_motor_id_list
    foot_id = anymal_foot_id
else:
    init_new_pos = spot_init_new_pos
    motor_id_list = spot_motor_id_list
    foot_id = spot_foot_id

MAX_STEPS = 6000


def get_data_from_sim(robot, motor_id_list):
    global get_last_vel
    get_orientation = []
    get_matrix = []
    get_velocity = []
    get_invert = []
    imu_data = [0] * 10 # [ax ay az qx qy qz qw wx wy wz]
    leg_data = [0] * 24 # [joint_position x 12,joint_velocity x 12] 

    pose_orn = p.getBasePositionAndOrientation(robot)

    for i in range(4):
        get_orientation.append(pose_orn[1][i])
    get_velocity = p.getBaseVelocity(robot)
    get_invert = p.invertTransform(pose_orn[0], pose_orn[1]) # from world to base
    get_matrix = p.getMatrixFromQuaternion(get_invert[1])

    # IMU data
    imu_data[3] = pose_orn[1][0]
    imu_data[4] = pose_orn[1][1]
    imu_data[5] = pose_orn[1][2]
    imu_data[6] = pose_orn[1][3]

    # w_base = R * w_world
    imu_data[7] = get_matrix[0] * get_velocity[1][0] + get_matrix[1] * get_velocity[1][1] + get_matrix[2] * get_velocity[1][2]
    imu_data[8] = get_matrix[3] * get_velocity[1][0] + get_matrix[4] * get_velocity[1][1] + get_matrix[5] * get_velocity[1][2]
    imu_data[9] = get_matrix[6] * get_velocity[1][0] + get_matrix[7] * get_velocity[1][1] + get_matrix[8] * get_velocity[1][2]

    # calculate the acceleration of the robot
    linear_X = (get_velocity[0][0] - get_last_vel[0]) * freq
    linear_Y = (get_velocity[0][1] - get_last_vel[1]) * freq
    linear_Z = 9.8 + (get_velocity[0][2] - get_last_vel[2]) * freq

    imu_data[0] = get_matrix[0] * linear_X + get_matrix[1] * linear_Y + get_matrix[2] * linear_Z
    imu_data[1] = get_matrix[3] * linear_X + get_matrix[4] * linear_Y + get_matrix[5] * linear_Z
    imu_data[2] = get_matrix[6] * linear_X + get_matrix[7] * linear_Y + get_matrix[8] * linear_Z

    # joint data
    joint_state = p.getJointStates(robot, motor_id_list)
    leg_data[0:12] = [joint_state[0][0], joint_state[1][0], joint_state[2][0],
                      joint_state[3][0], joint_state[4][0], joint_state[5][0],
                      joint_state[6][0], joint_state[7][0], joint_state[8][0],
                      joint_state[9][0], joint_state[10][0], joint_state[11][0]]

    leg_data[12:24] = [joint_state[0][1], joint_state[1][1], joint_state[2][1],
                       joint_state[3][1], joint_state[4][1], joint_state[5][1],
                       joint_state[6][1], joint_state[7][1], joint_state[8][1],
                       joint_state[9][1], joint_state[10][1], joint_state[11][1]]
    com_velocity = [get_velocity[0][0],
                    get_velocity[0][1], get_velocity[0][2]]
    # get_last_vel.clear()
    get_last_vel = []
    get_last_vel = com_velocity

    return imu_data, leg_data, pose_orn[0]


def reset_robot(quad_model, quad_controller, pos):

    p.resetBasePositionAndOrientation(quad_model, pos, [0, 0, 0, 1])
    p.resetBaseVelocity(quad_model, [0, 0, 0], [0, 0, 0])
    
    for j in range(12):
        p.resetJointState(quad_model, motor_id_list[j], init_new_pos[j], 0)

    
    for j in range(12):
        force = 0
        p.setJointMotorControl2(quad_model, motor_id_list[j], p.VELOCITY_CONTROL, force=force)

    for _ in range(100):
        p.stepSimulation()
        imu_data, leg_data, _ = get_data_from_sim(quad_model, motor_id_list)
        quad_controller.pre_work(imu_data, leg_data)
        # time.sleep(0.02)

    # time.sleep(3)


def change_dynamics(quad_model, foot_id):

    for i in range(4):
        p.changeDynamics(quad_model, foot_id[i], spinningFriction=spinningFriction, lateralFriction=lateralFriction)


def init_simulator():
    
    # 图形界面debug开关
    # p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
    p.resetSimulation()
    p.setTimeStep(1.0/freq)
    p.setGravity(0, 0, -9.8)
    
    p.resetDebugVisualizerCamera(0.2, 45, -30, [1, -1, 1])

    planeShape = p.createCollisionShape(shapeType=p.GEOM_PLANE)
    ground_id = p.createMultiBody(0, planeShape)
    p.resetBasePositionAndOrientation(ground_id, [0, 0, 0], [0, 0, 0, 1])
    p.changeDynamics(ground_id, -1, lateralFriction=lateralFriction)

    slope = p.loadURDF("quadruped_model/slope/slope.xml", [-0.5, -0.5, 0], useFixedBase=True)
    p.changeDynamics(slope, -1, lateralFriction=lateralFriction)
    # time.sleep(2)

def run(quad, quad_controller, actions):

    # get data from simulator
    imu_data, leg_data, base_pos = get_data_from_sim(quad, motor_id_list)
    
    quad_controller.set_robot_vel(actions)
    # call cpp function to calculate mpc tau
    t0 = time.time()
    tau = quad_controller.run(imu_data, leg_data)
    # print(time.time()-t0)
    # set tau to simulator
    
    p.setJointMotorControlArray(bodyUniqueId=quad,
                                jointIndices=motor_id_list,
                                controlMode=p.TORQUE_CONTROL,
                                forces=tau[0:12])
    # tau_arrays.append(tau)
    # imu_array.append(imu_data)
    # leg_array.append(leg_data)
    # reset visual cam follow the robot
    p.resetDebugVisualizerCamera(1.8, 45, -30, [base_pos[0], base_pos[1], 0.4])
    
    p.stepSimulation()
    time.sleep(0.001)
    return


# def draw_foot_traj(robot):
#     for i in range(4):
#         foot_pos = np.array(p.getLinkState(robot, foot_id[i])[0])
#         p.addUserDebugLine(pre_pos[i], foot_pos, lineColorRGB=[1, 0, 0], lifeTime=5, lineWidth=3)
#         pre_pos[i] = foot_pos


def main():

    cnt = 0
    reset_flag = p.readUserDebugParameter(reset)
    while 1:
    # for i in range(MAX_STEPS):
        # check reset button state
        if(reset_flag < p.readUserDebugParameter(reset)):
            reset_flag = p.readUserDebugParameter(reset)
            cnt = 0
            # reset_robot(mini_cheetah, mini_cheetah_runner, [0, 1, 0.4])
            # reset_robot(a1, a1_runner, [0, -1, 0.4])
            reset_robot(robot, robot_runner, [0, 0, 0.6])

        # stand 0 / locomotion 2
        if cnt < 300: 
            # passive
            robot_runner.set_control_mode(0)

        elif cnt == 300:
            # stand
            robot_runner.set_control_mode(1)

        elif cnt == 700:
            # locomotion
            robot_runner.set_control_mode(2)

        elif cnt == 900:
           # set to trot
            robot_runner.set_gait_type(9)

        x = p.readUserDebugParameter(liner_x)
        y = p.readUserDebugParameter(liner_y)
        yaw = p.readUserDebugParameter(yaw_rate)
        action = [x, y, yaw, 0]
        # action = joystick.get_command()
        run(robot, robot_runner, action)
        cnt += 1
        if cnt > 99999999:
            cnt = 99999999
            cnt = 0


if __name__ == '__main__':
    tau_arrays = []
    imu_array = []
    leg_array = []
    terrain = "plane"
    lateralFriction = 1.0
    spinningFriction = 0.065
    freq = 500.0
    # joystick = JoystickInterface()
    
    p.connect(p.GUI)
    reset = p.addUserDebugParameter("reset", 1, 0, 0)

    # 最大速度 addUserDebugParameter（“滑块名”， 最小值， 最大值, 默认值）
    liner_x = p.addUserDebugParameter("liner_x", -1.5, 1.5, 0)
    liner_y = p.addUserDebugParameter("liner_y", -0.6, 0.6, 0)
    yaw_rate = p.addUserDebugParameter("yaw", -1, 1, 0)



    
    init_simulator()
    
    # mini_cheetah = 0
    # a1 = 1
    # anymal_b = 3

    robot_runner = robot_controller(1)    
    robot = p.loadURDF("quadruped_model/a1/a1.urdf", [0, 0, 0.4], useFixedBase=False)

    # pre_pos = np.zeros((4, 3))

    # for i in range(4):
    #     pre_pos[i] = p.getLinkState(robot, foot_id[i])[0]

    change_dynamics(robot, foot_id)

    reset_robot(robot, robot_runner, [0, 0, 0.35])

    main()

    x = np.linspace(-100,100,MAX_STEPS)
    tau_arrays = np.array(tau_arrays)
    imu_array = np.array(imu_array)
    leg_array = np.array(leg_array)

    # tau
    figure1 = plt.figure()
    for i in range(4):
        plt.subplot(4, 1, i+1)
        plt.plot(x, tau_arrays[:, 3*i], label='abad')
        plt.plot(x, tau_arrays[:, 3*i+1], label='hip')
        plt.plot(x, tau_arrays[:, 3*i+2], label='knee')
        plt.legend()

    # imu omega & acceleration
    # figure2 = plt.figure()
    # plt.subplot(2, 1, 1)
    # plt.plot(x, imu_array[:, 0], label='ax')
    # plt.plot(x, imu_array[:, 1], label='ay')
    # plt.plot(x, imu_array[:, 2], label='az')
    # plt.legend()

    # plt.subplot(2, 1, 2)
    # plt.plot(x, imu_array[:, 7], label='wx')
    # plt.plot(x, imu_array[:, 8], label='wy')
    # plt.plot(x, imu_array[:, 9], label='wz')
    # plt.legend()

    # fig3 = plt.figure()
    # for i in range(12):
    #     plt.subplot(4, 3, i + 1)
    #     plt.plot(x, leg_array[:, i])

    plt.show()
