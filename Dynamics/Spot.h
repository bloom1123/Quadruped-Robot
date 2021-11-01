#ifndef PROJECT_SPOT_H
#define PROJECT_SPOT_H

#include "FloatingBaseModel.h"
#include "Quadruped.h"

template <typename T>
Quadruped<T> buildSpot() {
  // Spot<T> quadruped;
  Quadruped<T> quadruped;
  quadruped._robotType = RobotType::MINI_CHEETAH;

  quadruped._bodyMass = 16.0;
  quadruped._bodyLength = 0.29785 * 2;
  quadruped._bodyWidth = 0.055 * 2;
  quadruped._bodyHeight = 0.05 * 2;
  quadruped._abadGearRatio = 6;
  quadruped._hipGearRatio = 6;
  quadruped._kneeGearRatio = 9.33;
  quadruped._abadLinkLength = 0.110945;
  quadruped._hipLinkLength = 0.3214;
  quadruped._kneeLinkY_offset = 0.000;
  quadruped._kneeLinkLength = 0.37;
  quadruped._maxLegLength = 0.6;

  quadruped._abadLocation = Vec3<T>(quadruped._bodyLength, quadruped._bodyWidth, 0) * 0.5;


  return quadruped;
}

#endif  // PROJECT_LAIKAGO_H
