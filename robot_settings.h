#ifndef _ROBOTSETTINGS_H
#define _ROBOTSETTINGS_H


#include <iostream>
#include "cppTypes.h"

// Vec3<float> Kd_joint_default_t(1, 0.2, 0.2);
// Vec3<float> Kp_joint_default_t(3, 3, 3);

struct UserParameters{

    float bo_height;
    float yawmaxfs;
    float rollmaxfs;
    float xmaxfs;
    float ymaxfs;
    float pitchdesin;
    int cmpc_gait;
    float cmpc_bonus_swing;
    float coef_fil;
    float det_terrn;
    bool use_wbc;
    float ori_pla_body;
    float bo_hei_jum;
    float muest;
    float shiftleg_x;
    float shiftleg_y;
    float gait_period_time;
    float swing_height;
    float mass;

    // use for wbc
    Vec3<float> Kd_joint;
    Vec3<float> Kp_joint;

    Vec3<float> Kp_body;
    Vec3<float> Kd_body;

    Vec3<float> Kp_foot;
    Vec3<float> Kd_foot;

    Vec3<float> Kp_ori;
    Vec3<float> Kd_ori;
    // end wbc

    // only for FSM_Stand_Up
    Vec3<float> kp_stand_up;
    Vec3<float> kd_stand_up;

    // for MPC
    Vec3<float> kp_swing_mpc;
    Vec3<float> kd_swing_mpc;

    Vec3<float> kp_stand_mpc;
    Vec3<float> kd_stand_mpc;

    Vec13<double> Qmat;

    UserParameters(float bo_height_default = 0.23f,
                    float yawmaxfs_default = 1.2f,
                    float rollmaxfs_default = 0.0,
                    float xmaxfs_default = 0.6f,
                    float ymaxfs_default = 1.0f,
                    float pitchdesin_default = 0.0f,
                    int cmpc_gait_default = 4,
                    float cmpc_bonus_swing_default = 0.0f,
                    float coef_fil_default = 0.4f,
                    float det_terrn_default = 1.0f,
                    bool use_wbc_default = false,
                    float ori_pla_body_default = 1.0f,
                    float bo_hei_jum_default = 0.25f,
                    float muest_default = 0.45f,
                    float shiftleg_x_default = 0.0f,
                    float shiftleg_y_default = 0.0f,
                    float gait_period_time_default = 0.22f,
                    float swing_height_default = 0.08,
                    float mass_default = 20.0)
    {
        bo_height = bo_height_default;
        yawmaxfs = yawmaxfs_default;
        rollmaxfs = rollmaxfs_default;
        xmaxfs = xmaxfs_default;
        ymaxfs = ymaxfs_default;
        pitchdesin = pitchdesin_default;
        cmpc_gait = cmpc_gait_default;
        cmpc_bonus_swing = cmpc_bonus_swing_default;
        coef_fil = coef_fil_default;
        det_terrn = det_terrn_default;
        use_wbc = use_wbc_default;
        ori_pla_body = ori_pla_body_default;
        bo_hei_jum = bo_hei_jum_default;
        muest = muest_default;
        shiftleg_x = shiftleg_x_default;
        shiftleg_y = shiftleg_y_default;
        gait_period_time = gait_period_time_default;
        swing_height = swing_height_default;
        mass = mass_default;

        Kd_joint << 1, 0.2, 0.2;
        Kp_joint << 3, 3, 3;

        Kp_body << 100, 100, 100;
        Kd_body << 10, 10, 10;

        Kp_foot << 500, 500, 500;
        Kd_foot << 10, 10, 10;

        Kp_ori << 100, 100, 100;
        Kd_ori << 10, 10, 10;

        kp_stand_up << 500, 500, 500;
        kd_stand_up << 8, 8, 8;

        kp_stand_mpc << 0, 0, 0;
        kd_stand_mpc << 7, 7, 7;

        kp_swing_mpc << 700, 700, 150;
        kd_swing_mpc << 7, 7, 7;

        // Qmat << 1.0, 1.0, 1.0, 1.0, 3.0, 3.0, 10.0, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2; //default
        // Qmat << 1.2, 1.2, 1.2, 1.2, 2.5, 2.5, 10.0, 0, 0, 0.5, 0.2, 0.2, 0.2; // spot
    };
};


struct ControlParameters{
    int control_mode;
    float controller_dt;
    float imu_process_noise_position;
    float imu_process_noise_velocity;
    float foot_sensor_noise_position;
    float foot_process_noise_position;
    float foot_sensor_noise_velocity;
    float foot_height_sensor_noise;

    ControlParameters(int control_mode_default = 0,
    float controller_dt_default = 0.002,
    float imu_process_noise_position_default = 0.02,
    float imu_process_noise_velocity_default = 0.02,
    float foot_sensor_noise_position_default = 0.001,
    float foot_process_noise_position_default = 0.002,
    float foot_sensor_noise_velocity_default = 0.1,
    float foot_height_sensor_noise_default = 0.001)
    {
        control_mode = control_mode_default;
        controller_dt = controller_dt_default;
        imu_process_noise_position = imu_process_noise_position_default;
        imu_process_noise_velocity = imu_process_noise_velocity_default;
        foot_sensor_noise_position = foot_sensor_noise_position_default;
        foot_process_noise_position = foot_process_noise_position_default;
        foot_sensor_noise_velocity = foot_sensor_noise_velocity_default;
        foot_height_sensor_noise = foot_height_sensor_noise_default;
    }
};

struct Test
{
    int a;
    int b;
    Test(int temp_a=1, int temp_b=2){
        a = temp_a;
        b = temp_b;
    };
};


#endif