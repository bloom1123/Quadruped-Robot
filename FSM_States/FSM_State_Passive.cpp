#include "FSM_State_Passive.h"

template <typename T>
FSM_State_Passive<T>::FSM_State_Passive(ControlFSMData<T>* _controlFSMData):
    FSM_State<T>(_controlFSMData, FSM_StateName::PASSIVE, "PASSIVE") 
{
    this->checkSafeOrientation = false;
    this->checkPDesFoot = false;
    this->checkForceFeedForward = false;
}

template <typename T>
void FSM_State_Passive<T>::onEnter() {
  // Default is to not transition
    this->nextStateName = this->stateName;

  // Reset the transition data
    this->transitionData.zero();
}

template <typename T>
void FSM_State_Passive<T>::run() {
    this->transitionData.done = true;
}


template <typename T>
FSM_StateName FSM_State_Passive<T>::checkTransition() {
    // std::cout << "--- --- --- check 0 " << std::endl;
    this->nextStateName = this->stateName;
    // std::cout << "--- --- --- check 1 " << std::endl;
    iter++;

    switch ((int)this->_data->controlParameters->control_mode) {
        
        case K_PASSIVE: 
        // std::cout << "set control mode to PASSIVE " << std::endl;
        break;

        case K_STAND_UP:
        this->nextStateName = FSM_StateName::STAND_UP;
        std::cout << "set control mode to STAND_UP " << std::endl;
        break;

        default:
        std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                    << K_PASSIVE << " to "
                    << this->_data->controlParameters->control_mode << std::endl;
    }

    // Get the next state
    return this->nextStateName;
}


template <typename T>
TransitionData<T> FSM_State_Passive<T>::transition() {
  
    this->transitionData.done = true;
    return this->transitionData;

}


template <typename T>
void FSM_State_Passive<T>::onExit() {
}

template class FSM_State_Passive<float>;