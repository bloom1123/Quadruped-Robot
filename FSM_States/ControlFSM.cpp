#include "ControlFSM.h"

template <typename T>
ControlFSM<T>::ControlFSM(Quadruped<T>* _quadruped,
                          LegController<T>* _legController,
                          StateEstimatorContainer<T>* _stateEstimator,
                          DesiredStateCommand* _desiredStateCommand,
                          ControlParameters* controlParameters,
                          UserParameters* userParameters)
{  

  data._quadruped = _quadruped;
  data._legController = _legController;
  data.controlParameters = controlParameters;
  data._desiredStateCommand = _desiredStateCommand;
  data.userParameters = userParameters;
  data._stateEstimator = _stateEstimator;

  statesList.invalid = nullptr;
  statesList.passive = new FSM_State_Passive<T>(&data);
  statesList.standUp = new FSM_State_StandUp<T>(&data);
  statesList.locomotion = new FSM_State_Locomotion<T>(&data);
  safetyChecker = new SafetyChecker<T>(&data);
  
  initialize();
}

template <typename T>
void ControlFSM<T>::initialize() 
{
    // Initialize a new FSM State with the control data
  currentState = statesList.passive;

  currentState->onEnter();

  nextState = currentState;

  operatingMode = FSM_OperatingMode::NORMAL;

}

template <typename T>
void ControlFSM<T>::runFSM()
{
  operatingMode = safetyPreCheck();
  if (operatingMode != FSM_OperatingMode::ESTOP) 
  { 
    if (operatingMode == FSM_OperatingMode::NORMAL) {
      nextStateName = currentState->checkTransition();
      if (nextStateName != currentState->stateName) {
        operatingMode = FSM_OperatingMode::TRANSITIONING;
        nextState = getNextState(nextStateName);
      } else {
        currentState->run();
      }
    }
    if (operatingMode == FSM_OperatingMode::TRANSITIONING) {
      transitionData = currentState->transition();
      safetyPostCheck();
      if (transitionData.done) {
        currentState->onExit();
        currentState = nextState;
        currentState->onEnter();
        operatingMode = FSM_OperatingMode::NORMAL;
      }
    } else {
      safetyPostCheck();
    }

  } else { // if ESTOP
		currentState = statesList.passive;
		currentState->onEnter();
		nextStateName = currentState->stateName;
  }
  // Increase the iteration counter
  iter++;
}


template <typename T>
FSM_OperatingMode ControlFSM<T>::safetyPreCheck() {
  // Check for safe orientation if the current state requires it
  if (currentState->checkSafeOrientation) {
    if (!safetyChecker->checkSafeOrientation()) {
      operatingMode = FSM_OperatingMode::ESTOP;
      std::cout << "broken: Orientation Safety Ceck FAIL" << std::endl;
    }
  }

  return operatingMode;
}


template <typename T>
FSM_OperatingMode ControlFSM<T>::safetyPostCheck() {
  // Check for safe desired foot positions
  if (currentState->checkPDesFoot) {
    safetyChecker->checkPDesFoot();
  }

  // Check for safe desired feedforward forces
  if (currentState->checkForceFeedForward) {
    safetyChecker->checkForceFeedForward();
  }

  // Default is to return the current operating mode
  return operatingMode;
}


template <typename T>
FSM_State<T>* ControlFSM<T>::getNextState(FSM_StateName stateName) {
  // Choose the correct FSM State by enumerated state name
  switch (stateName) {
    case FSM_StateName::INVALID:
      return statesList.invalid;

    case FSM_StateName::PASSIVE:
      return statesList.passive;

    case FSM_StateName::STAND_UP:
      return statesList.standUp;

    case FSM_StateName::LOCOMOTION:
      return statesList.locomotion;

    default:
      return statesList.invalid;
  }
}

template class ControlFSM<float>;