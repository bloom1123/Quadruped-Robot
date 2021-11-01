#ifndef CONTROLFSMDATA_H
#define CONTROLFSMDATA_H

#include "../robot_settings.h"
#include "../Dynamics/Quadruped.h"
#include "../Controllers/LegController.h"
#include "../Controllers/StateEstimatorContainer.h"
#include "../Controllers/DesiredStateCommand.h"
template <typename T>
struct ControlFSMData {
  Quadruped<T>* _quadruped;
  LegController<T>* _legController;
  StateEstimatorContainer<T>* _stateEstimator;
  DesiredStateCommand* _desiredStateCommand;
  ControlParameters* controlParameters;
  UserParameters* userParameters;
};

template struct ControlFSMData<double>;
template struct ControlFSMData<float>;

#endif  // CONTROLFSM_H