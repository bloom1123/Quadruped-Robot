#include "FSM_State_Locomotion.h"
// #include "../Controllers/WBC_Ctrl/WBC_Ctrl.h"
template <typename T>
FSM_State_Locomotion<T>::FSM_State_Locomotion(ControlFSMData<T>* _controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::LOCOMOTION, "LOCOMOTION")
{
  cMPCOld = new ConvexMPCLocomotion(_controlFSMData->controlParameters->controller_dt,
                                    _controlFSMData->userParameters->gait_period_time
  );

  this->turnOnAllSafetyChecks();
  // Turn off Foot pos command since it is set in WBC as operational task
  this->checkPDesFoot = false;

}


template <typename T>
void FSM_State_Locomotion<T>::onEnter() {
  this->nextStateName = this->stateName;
  this->transitionData.zero();
  cMPCOld->initialize();
  std::cout << "[FSM LOCOMOTION] On Enter\n" << std::endl;
}

template <typename T>
void FSM_State_Locomotion<T>::run() {
  // Call the locomotion control logic for this iteration
  cMPCOld->run<T>(*this->_data);

  // Vec3<T> pDes_backup[4];
  // Vec3<T> vDes_backup[4];
  // Mat3<T> Kp_backup[4];
  // Mat3<T> Kd_backup[4];

  // for(int leg(0); leg<4; ++leg){
  //   pDes_backup[leg] = this->_data->_legController->commands[leg].pDes;
  //   vDes_backup[leg] = this->_data->_legController->commands[leg].vDes;
  //   Kp_backup[leg] = this->_data->_legController->commands[leg].kpCartesian;
  //   Kd_backup[leg] = this->_data->_legController->commands[leg].kdCartesian;
  // }

  // if(this->_data->userParameters->use_wbc > 0.9){
  //   _wbc_data->pBody_des = cMPCOld->pBody_des;
  //   _wbc_data->vBody_des = cMPCOld->vBody_des;
  //   _wbc_data->aBody_des = cMPCOld->aBody_des;

  //   _wbc_data->pBody_RPY_des = cMPCOld->pBody_RPY_des;
  //   _wbc_data->vBody_Ori_des = cMPCOld->vBody_Ori_des;
    
  //   for(size_t i(0); i<4; ++i){
  //     _wbc_data->pFoot_des[i] = cMPCOld->pFoot_des[i];
  //     _wbc_data->vFoot_des[i] = cMPCOld->vFoot_des[i];
  //     _wbc_data->aFoot_des[i] = cMPCOld->aFoot_des[i];
  //     _wbc_data->Fr_des[i] = cMPCOld->Fr_des[i];
  //     _wbc_data->muv[i] = cMPCOld->muv[i];
  //     _wbc_data->Rpla[i] = cMPCOld->Rpla[i];
  //     //pretty_print((_wbc_data->Fr_des[i]), std::cout, "Fr entered to wbc_data");
  //   }
  //   _wbc_data->contact_state = cMPCOld->contact_state;

  //   _wbc_ctrl->run(_wbc_data, *this->_data);
  // }
  // for(int leg(0); leg<4; ++leg){
  //   //this->_data->_legController->commands[leg].pDes = pDes_backup[leg];
  //   this->_data->_legController->commands[leg].vDes = vDes_backup[leg];
  //   //this->_data->_legController->commands[leg].kpCartesian = Kp_backup[leg];
  //   this->_data->_legController->commands[leg].kdCartesian = Kd_backup[leg];
  // }
  
}


template <typename T>
FSM_StateName FSM_State_Locomotion<T>::checkTransition() {
  // Get the next state
  iter++;

  // Switch FSM control mode
  if(locomotionSafe()) {
    switch ((int)this->_data->controlParameters->control_mode) {
      case K_LOCOMOTION:
        break;

      case K_PASSIVE:
        // Requested change to BALANCE_STAND
        this->nextStateName = FSM_StateName::PASSIVE;

        // Transition time is immediate
        this->transitionDuration = 0.0;

        break;

      case K_STAND_UP:
        this->nextStateName = FSM_StateName::STAND_UP;
        this->transitionDuration = 0.;
        break;

      default:
        std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                  << K_LOCOMOTION << " to "
                  << this->_data->controlParameters->control_mode << std::endl;
    }
  }

  // Return the next state name to the FSM
  return this->nextStateName;
}


template <typename T>
TransitionData<T> FSM_State_Locomotion<T>::transition() {
  // Switch FSM control mode
  switch (this->nextStateName) {
    case FSM_StateName::PASSIVE:
      this->turnOffAllSafetyChecks();

      this->transitionData.done = true;

      break;

    case FSM_StateName::STAND_UP:
      this->transitionData.done = true;
      break;

    default:
      std::cout << "[CONTROL FSM] Something went wrong in transition"
                << std::endl;
  }

  // Return the transition data to the FSM
  return this->transitionData;
}


template<typename T>
bool FSM_State_Locomotion<T>::locomotionSafe() {
  auto& seResult = this->_data->_stateEstimator->getResult();

  const T max_roll = 40;
  const T max_pitch = 40;

  if(std::fabs(seResult.rpy[0]) > ori::deg2rad(max_roll)) {
    printf("Unsafe locomotion: roll is %.3f degrees (max %.3f)\n", ori::rad2deg(seResult.rpy[0]), max_roll);
    return true;
  }

  if(std::fabs(seResult.rpy[1]) > ori::deg2rad(max_pitch)) {
    printf("Unsafe locomotion: pitch is %.3f degrees (max %.3f)\n", ori::rad2deg(seResult.rpy[1]), max_pitch);
    return true;
  }

  for(int leg = 0; leg < 4; leg++) {
    auto p_leg = this->_data->_legController->datas[leg].p;
    if(p_leg[2] > 0) {
      printf("Unsafe locomotion: leg %d is above hip (%.3f m)\n", leg, p_leg[2]);
      return true;
    }

    if(std::fabs(p_leg[1] > 0.25)) {
      printf("true locomotion: leg %d's y-position is bad (%.3f m)\n", leg, p_leg[1]);
      return true;
    }

    auto v_leg = this->_data->_legController->datas[leg].v.norm();
    if(std::fabs(v_leg) > 15.) {
      printf("Unsafe locomotion: leg %d is moving too quickly (%.3f m/s)\n", leg, v_leg);
      return true;
    }
  }

  return true;

}


template <typename T>
void FSM_State_Locomotion<T>::onExit() {
  // Nothing to clean up when exiting
  iter = 0;
}

template class FSM_State_Locomotion<float>;