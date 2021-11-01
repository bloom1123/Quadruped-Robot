
#include "FSM_State_StandUp.h"

template <typename T>
FSM_State_StandUp<T>::FSM_State_StandUp(ControlFSMData<T>* _controlFSMData): 
    FSM_State<T>(_controlFSMData, FSM_StateName::STAND_UP, "STAND_UP"),
    _ini_foot_pos(4)
{
  
  this->checkSafeOrientation = false;
  this->checkPDesFoot = false;
  this->checkForceFeedForward = false;
}


template <typename T>
void FSM_State_StandUp<T>::onEnter() {
  // Default is to not transition
  this->nextStateName = this->stateName;
  this->transitionData.zero();

  iter = 0;

  for(size_t leg(0); leg<4; ++leg){
    _ini_foot_pos[leg] = this->_data->_legController->datas[leg].p;
  }

}


template <typename T>
void FSM_State_StandUp<T>::run() {

  if(this->_data->_quadruped->_robotType == RobotType::MINI_CHEETAH) {
    T hMax = this->_data->userParameters->bo_height;
    // T hMax = 0.5;
    T progress = 2 * iter * this->_data->controlParameters->controller_dt;

    if (progress > 1.){ progress = 1.; }

    for(int i = 0; i < 4; i++) {
      Vec3<T> kp = this->_data->userParameters->kp_stand_up;
      Vec3<T> kd = this->_data->userParameters->kd_stand_up;
      // std::cout << "kp: " << kp << std::endl;
      // std::cout << "kd: " << kd << std::endl;
      this->_data->_legController->commands[i].kpCartesian = kp.asDiagonal();
      this->_data->_legController->commands[i].kdCartesian = kd.asDiagonal();
      // this->_data->_legController->commands[i].kpCartesian = Vec3<T>(500, 500, 500).asDiagonal();
      // this->_data->_legController->commands[i].kdCartesian = Vec3<T>(8, 8, 8).asDiagonal();

      this->_data->_legController->commands[i].pDes = _ini_foot_pos[i];
      this->_data->_legController->commands[i].pDes[2] = 
        progress*(-hMax) + (1. - progress) * _ini_foot_pos[i][2];
    }
  }
}


template <typename T>
FSM_StateName FSM_State_StandUp<T>::checkTransition() {
  this->nextStateName = this->stateName;
  iter++;

  // Switch FSM control mode
  switch ((int)this->_data->controlParameters->control_mode) {
    case K_STAND_UP:
      // std::cout << "set control mode to STAND_UP" << std::endl;
      break;

    case K_LOCOMOTION:
      this->nextStateName = FSM_StateName::LOCOMOTION;
      std::cout << "set control mode to LOCOMOTION" << std::endl;
      break;

    case K_PASSIVE:  // normal c
      this->nextStateName = FSM_StateName::PASSIVE;
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
TransitionData<T> FSM_State_StandUp<T>::transition() {
  // Finish Transition
  switch (this->nextStateName) {
    case FSM_StateName::PASSIVE:  // normal
      this->transitionData.done = true;
      break;

    case FSM_StateName::LOCOMOTION:
      this->transitionData.done = true;
      break;

    default:
      std::cout << "[CONTROL FSM] Something went wrong in transition"
                << std::endl;
  }

  // Return the transition data to the FSM
  return this->transitionData;
}


template <typename T>
void FSM_State_StandUp<T>::onExit() {
}

template class FSM_State_StandUp<float>;