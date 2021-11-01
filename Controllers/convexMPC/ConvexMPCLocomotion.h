#ifndef CHEETAH_SOFTWARE_CONVEXMPCLOCOMOTION_H
#define CHEETAH_SOFTWARE_CONVEXMPCLOCOMOTION_H

#include "FootSwingTrajectory.h"
#include "../../FSM_States/ControlFSMData.h"
#include "../../cppTypes.h"
#include "Gait.h"

#include <cstdio>

using Eigen::Array4f;
using Eigen::Array4i;
using Eigen::Dynamic;

template<typename T>
struct CMPC_Result {
  LegControllerCommand<T> commands[4];
  Vec4<T> contactPhase;
};


class ConvexMPCLocomotion {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ConvexMPCLocomotion(float _dt, float gait_period_time);
  void initialize();
  template<typename T>
  void run(ControlFSMData<T>& data);
  // bool currently_jumping = false;

  // Vec3<float> pBody_des;
  // Vec3<float> vBody_des;
  // Vec3<float> aBody_des;
  // Vec3<float> pBody_RPY_des;
  // Vec3<float> vBody_Ori_des;

  Vec3<float> pFoot_des[4];
  Vec3<float> vFoot_des[4];
  Vec3<float> aFoot_des[4];
  Vec3<float> Fr_des[4];
  Vec4<float> contact_state;
  Eigen::Matrix<float,Dynamic,Dynamic> Wpla;
  Eigen::Matrix<float,Dynamic,Dynamic> Wplainv;
  float heightoffCoM = 0;
  float heightoffCoM_prev = 0;
  Eigen::Matrix<float,4,1> zfeet;
  Eigen::Matrix<float,3,1> apla;
  Eigen::Matrix<float,3,1> apla2;
;
  Eigen::Matrix<float,3,3> Rpla[4];
  Eigen::Matrix<float,3,3> Rpla2[4];
  float muv[4] = {0.4f,0.4f,0.4f,0.4f};
  Vec3<float> rpyplane;
  Vec3<float> rpydesfin;
  float pestCoMx;
  float pestCoMy;

private:
  void _SetupCommand(ControlFSMData<float> & data);
  float _yaw_turn_rate;
  float _yaw_des = 0.;
  float _roll_turn_rate;
  float _roll_des = 0.;
  float _pitch_des = 0.;
  float _x_vel_des = 0.;
  float _y_vel_des = 0.;
  float _body_height = 0.29;
  void recompute_timing(float gait_period_time);
  void updateMPCIfNeeded(Gait *gait, int* mpcTable, ControlFSMData<float>& data, bool omniMode, int horLength);
  void solveDenseMPC(int *mpcTable, ControlFSMData<float> &data, int horLength);
  //void solveSparseMPC(int *mpcTable, ControlFSMData<float> &data);
  //void initSparseMPC();
  int iterationsBetweenMPC;
  int horizonLength;
  float dt;
  float dtMPC;
  int iterationCounter = 0;
  Vec3<float> f_ff[4];
  Vec4<float> swingTimes;
  FootSwingTrajectory<float> footSwingTrajectories[4];
  OffsetDurationGait trotting, bounding, pronking, jumping, galloping, standing, trotRunning, walking, walking2, pacing, oneaerial;
  MixedFrequncyGait random, random2;
  Mat3<float> Kp, Kd, Kp_stance, Kd_stance;
  bool firstRun = true;
  bool firstSwing[4];
  float swingTimeRemaining[4];
  float stand_traj[6];
  int current_gait;
  int previous_gait;
  int gaitNumber;
  bool bswitchgait=false;
  Vec3<float> world_position_desired;
  Vec3<float> pFoot[4];
  CMPC_Result<float> result;
  float trajAll[13*36];
  float x_vel_cmd, y_vel_cmd, yaw_vel_cmd;
  int stop_counter = 0;
  // vectorAligned<Vec12<double>> _sparseTrajectory;
};
#endif //CHEETAH_SOFTWARE_CONVEXMPCLOCOMOTION_H