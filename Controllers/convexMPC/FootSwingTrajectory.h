
#ifndef CHEETAH_SOFTWARE_FOOTSWINGTRAJECTORY_H
#define CHEETAH_SOFTWARE_FOOTSWINGTRAJECTORY_H

#include "../../cppTypes.h"

namespace Interpolate {
template <typename y_t, typename x_t>
y_t cubicBezier(y_t y0, y_t yf, x_t x) {

  y_t yDiff = yf - y0;
  x_t bezier = x * x * x + x_t(3) * (x * x * (x_t(1) - x));
  return y0 + bezier * yDiff;
}


template <typename y_t, typename x_t>
y_t cubicBezierFirstDerivative(y_t y0, y_t yf, x_t x) {

  y_t yDiff = yf - y0;
  x_t bezier = x_t(6) * x * (x_t(1) - x);
  return bezier * yDiff;
}


template <typename y_t, typename x_t>
y_t cubicBezierSecondDerivative(y_t y0, y_t yf, x_t x) {

  y_t yDiff = yf - y0;
  x_t bezier = x_t(6) - x_t(12) * x;
  return bezier * yDiff;
}

}  // namespace Interpolate


template<typename T>
class FootSwingTrajectory {
public:

  FootSwingTrajectory() {
    _p0.setZero();
    _pf.setZero();
    _p.setZero();
    _v.setZero();
    _a.setZero();
    _height = 0;
  }

  void setInitialPosition(Vec3<T> p0) {
    _p0 = p0;
  }

  void setFinalPosition(Vec3<T> pf) {
    _pf = pf;
  }


  void setHeight(T h) {
    _height = h;
  }

  void computeSwingTrajectoryBezier(T phase, T swingTime);

  Vec3<T> getPosition() {
    return _p;
  }

  Vec3<T> getVelocity() {
    return _v;
  }

  Vec3<T> getAcceleration() {
    return _a;
  }

private:
  Vec3<T> _p0, _pf, _p, _v, _a;
  T _height;
};


#endif //CHEETAH_SOFTWARE_FOOTSWINGTRAJECTORY_H
