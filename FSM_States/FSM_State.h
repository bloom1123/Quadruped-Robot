#ifndef FSM_State_H
#define FSM_State_H

#include "TransitionData.h"
#include "ControlFSMData.h"
#include <iostream>

#define K_PASSIVE 0
#define K_STAND_UP 1
#define K_LOCOMOTION 2
#define K_INVALID 100


enum class FSM_StateName {
  INVALID,
  PASSIVE,
  STAND_UP,
  LOCOMOTION
};


template <typename T>
class FSM_State
{
private:

public:

    FSM_State(ControlFSMData<T>* _controlFSMData, 
              FSM_StateName stateNameIn,
              std::string stateStringIn);

    virtual void onEnter() = 0;

    virtual void run() = 0;

    virtual FSM_StateName checkTransition() { return FSM_StateName::INVALID; }

    virtual TransitionData<T> transition() { return transitionData; }

    virtual void onExit() = 0;

    void turnOnAllSafetyChecks();
    void turnOffAllSafetyChecks();

    T transitionDuration;
    TransitionData<T> transitionData;
    ControlFSMData<T>* _data;
    
    bool checkSafeOrientation = false;
    bool checkPDesFoot = false;
    bool checkForceFeedForward = false;
    bool checkLegSingularity = false;

    FSM_StateName stateName; 
    FSM_StateName nextStateName;
    std::string stateString;
    
};

#endif  // FSM_State_H