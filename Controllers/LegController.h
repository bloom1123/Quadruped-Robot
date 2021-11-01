
#ifndef PROJECT_LEGCONTROLLER_H
#define PROJECT_LEGCONTROLLER_H

#include "../Dynamics/Quadruped.h"
#include "RobotLegState.h"

template <typename T>
struct LegControllerCommand {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LegControllerCommand() { zero(); }

  void zero();

  Vec3<T> tauFeedForward, forceFeedForward, qDes, qdDes, pDes, vDes;
  Mat3<T> kpCartesian, kdCartesian, kpJoint, kdJoint;
};

/*!
 * Data returned from the legs to the control code.
 * 腿部反馈给控制代码的关节实时数据
 */
template <typename T>
struct LegControllerData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LegControllerData() { zero(); }

  void setQuadruped(Quadruped<T>& quad) { quadruped = &quad; }

  void zero();

  Vec3<T> q, qd, p, v;
  Mat3<T> J;
  Vec3<T> tauEstimate;
  Quadruped<T>* quadruped;
};

/*!
 * Controller for 4 legs of a quadruped.  Works for both Mini Cheetah and
 * Cheetah 3
 */
template <typename T>
class LegController {
 public:
  LegController(Quadruped<T>& quad) : _quadruped(quad) {
    for (auto& data : datas) data.setQuadruped(_quadruped);
  }

  void zeroCommand();
  void edampCommand(RobotType robot, T gain);
  void updateData(LegData* legData);
  void updateCommand(LegCommand* legCommand);
  void setEnabled(bool enabled) { _legsEnabled = enabled; };
  void setMaxTorqueCheetah3(T tau) { _maxTorque = tau; }

  LegControllerCommand<T> commands[4];
  LegControllerData<T> datas[4];
  Quadruped<T>& _quadruped;
  bool _legsEnabled = false;
  T _maxTorque = 0;

  int myflags = 0;
};

template <typename T>
void computeLegJacobianAndPosition(Quadruped<T>& quad, Vec3<T>& q, Mat3<T>* J,
                                   Vec3<T>* p, int leg);

#endif  // PROJECT_LEGCONTROLLER_H
