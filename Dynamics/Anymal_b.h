#ifndef PROJECT_ANYMAY_B_H
#define PROJECT_ANYMAY_B_H

#include "FloatingBaseModel.h"
#include "Quadruped.h"
#include "../robot_settings.h"
template <typename T>
Quadruped<T> buildANYMAY_B(UserParameters* userParameters) {
  // Spot<T> quadruped;
  Quadruped<T> quadruped;
  quadruped._robotType = RobotType::MINI_CHEETAH;

  quadruped._bodyMass = 16.0;
  quadruped._bodyHeight = 0.05 * 2;
  quadruped._abadGearRatio = 6;
  quadruped._hipGearRatio = 6;
  quadruped._kneeGearRatio = 9.33;
  
  // body size
  quadruped._bodyLength = (0.277+0.0635) * 2;
  quadruped._bodyWidth = 0.116 * 2;
  // L0
  quadruped._abadLinkLength = 0.13;

  // L1
  quadruped._hipLinkLength = 0.25;

  // L2
  quadruped._kneeLinkY_offset = 0.000;
  quadruped._kneeLinkLength = 0.332;

  quadruped._maxLegLength = 0.5;

  quadruped._abadLocation = Vec3<T>(quadruped._bodyLength, quadruped._bodyWidth, 0) * 0.5;

  userParameters->bo_height = 0.4;
  userParameters->bo_hei_jum = 0.4;
  userParameters->swing_height = 0.12;
  userParameters->mass = 20;
  userParameters->xmaxfs = 1.0;
  userParameters->yawmaxfs = 1.0;
  userParameters->ymaxfs = 1.0;
  
  userParameters->kp_stand_up << 500, 500, 500;
  userParameters->kd_stand_up << 8, 8, 8;

  userParameters->kp_stand_mpc << 0, 0, 0;
  userParameters->kd_stand_mpc << 7, 7, 7;

  userParameters->kp_swing_mpc << 500, 500, 900;
  userParameters->kd_swing_mpc << 50, 50, 80;

  userParameters->Qmat << 1.0, 1.0, 1.0, 1.0, 3.0, 3.0, 15.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.2;


  return quadruped;
};

#endif  // PROJECT_LAIKAGO_H