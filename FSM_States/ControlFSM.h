#ifndef CONTROLFSM_H
#define CONTROLFSM_H


#include <iostream>
#include "FSM_State.h"
#include "FSM_State_Passive.h"
#include "FSM_State_StandUp.h"
#include "FSM_State_Locomotion.h"
#include "SafetyChecker.h"

// FSM control mode
 enum class FSM_OperatingMode { 
  NORMAL, 
  TRANSITIONING, 
  ESTOP, 
  EDAMP
};


// FSM state Lists
template <typename T>
struct FSM_StatesList {
  FSM_State<T>* invalid;
  FSM_State_Passive<T>* passive;
  FSM_State_StandUp<T>* standUp;
  FSM_State_Locomotion<T>* locomotion;
};

template <typename T>
class ControlFSM
{
private:
    
    FSM_OperatingMode operatingMode;
    TransitionData<T> transitionData;
    int iter = 0;

public:
    ControlFSM(Quadruped<T>* _quadruped,
               LegController<T>* _legController,
               StateEstimatorContainer<T>* _stateEstimator,
               DesiredStateCommand* _desiredStateCommand,
               ControlParameters* controlParameters,
               UserParameters* userParameters);

    void initialize();

    void runFSM();

    FSM_State<T>* getNextState(FSM_StateName stateName);
    
    FSM_OperatingMode safetyPreCheck();
    FSM_OperatingMode safetyPostCheck();
    ControlFSMData<T> data;

    SafetyChecker<T>* safetyChecker;
    FSM_StatesList<T> statesList;
    FSM_State<T>* currentState;
    FSM_State<T>* nextState;
    FSM_StateName nextStateName; 
};

#endif  // CONTROLFSM_H