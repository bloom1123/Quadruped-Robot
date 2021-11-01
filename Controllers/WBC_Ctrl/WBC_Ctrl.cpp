#include "WBC_Ctrl.h"
#include "TaskSet/SingleContact.hpp"
#include "TaskSet/BodyOriTask.hpp"
#include "TaskSet/BodyPosTask.hpp"
#include "TaskSet/LinkPosTask.hpp"

template<typename T>
WBC_Ctrl<T>::WBC_Ctrl(FloatingBaseModel<T> model):
    _full_config(cheetah::num_act_joint + 7),
    _tau_ff(cheetah::num_act_joint),
    _des_jpos(cheetah::num_act_joint),
    _des_jvel(cheetah::num_act_joint)
{
  _body_pos_task = new BodyPosTask<T>(&_model); //study 
  _body_ori_task = new BodyOriTask<T>(&_model);
  _foot_contact[0] = new SingleContact<T>(&_model, linkID::FR);
  _foot_contact[1] = new SingleContact<T>(&_model, linkID::FL);
  _foot_contact[2] = new SingleContact<T>(&_model, linkID::HR);
  _foot_contact[3] = new SingleContact<T>(&_model, linkID::HL);
  _foot_task[0] = new LinkPosTask<T>(&_model, linkID::FR);
  _foot_task[1] = new LinkPosTask<T>(&_model, linkID::FL);
  _foot_task[2] = new LinkPosTask<T>(&_model, linkID::HR);
  _foot_task[3] = new LinkPosTask<T>(&_model, linkID::HL);

  _iter = 0;
  _full_config.setZero();

  _model = model;
  _kin_wbc = new KinWBC<T>(cheetah::dim_config);

  _wbic = new WBIC<T>(cheetah::dim_config, &(_contact_list), &(_task_list));
  _wbic_data = new WBIC_ExtraData<T>();

  _wbic_data->_W_floating = DVec<T>::Constant(6, 0.1);

  _wbic_data->_W_rf = DVec<T>::Constant(12, 1.);

  _Kp_joint.resize(cheetah::num_leg_joint, 5.);
  _Kd_joint.resize(cheetah::num_leg_joint, 1.5);

  _state.q = DVec<T>::Zero(cheetah::num_act_joint);
  _state.qd = DVec<T>::Zero(cheetah::num_act_joint);

}


template<typename T>
void WBC_Ctrl<T>::run(void* input, ControlFSMData<T> & data){
  ++_iter;

  _UpdateModel(data._stateEstimator->getResult(), data._legController->datas);

  // Task & Contact Update
  _ContactTaskUpdate(input, data);

  // WBC Computation
  _ComputeWBC();

  // Update Leg Command
  _UpdateLegCMD(data);
}


template<typename T>
void WBC_Ctrl<T>::_UpdateModel(const StateEstimate<T> & state_est, 
    const LegControllerData<T> * leg_data){
  _state.bodyOrientation = state_est.orientation;
  _state.bodyPosition = state_est.position;
  for(size_t i(0); i<3; ++i){
    _state.bodyVelocity[i] = state_est.omegaBody[i];
    _state.bodyVelocity[i+3] = state_est.vBody[i];
    for(size_t leg(0); leg<4; ++leg){
      _state.q[3*leg + i] = leg_data[leg].q[i];
      _state.qd[3*leg + i] = leg_data[leg].qd[i];
      _full_config[3*leg + i + 6] = _state.q[3*leg + i];
    }
  }
  _model.setState(_state);
  _model.contactJacobians();
  _model.massMatrix();
  _model.generalizedGravityForce();
  _model.generalizedCoriolisForce();
  _A = _model.getMassMatrix();
  _grav = _model.getGravityForce();
  _coriolis = _model.getCoriolisForce();
  _Ainv = _A.inverse();
}


template <typename T>
void WBC_Ctrl<T>::_ComputeWBC() {
  // TEST
  _kin_wbc->FindConfiguration(_full_config, _task_list, _contact_list,
                              _des_jpos, _des_jvel);
                                                                                                               
  // WBIC
  _wbic->UpdateSetting(_A, _Ainv, _coriolis, _grav);

  _wbic->MakeTorque(_tau_ff, _wbic_data);
}


template<typename T>
void WBC_Ctrl<T>::_ContactTaskUpdate(void* input, ControlFSMData<T> & data){
  _input_data = static_cast<LocomotionCtrlData<T>* >(input);

  _ParameterSetup(data.userParameters);

  _CleanUp();
  _quat_des = ori::rpyToQuat(_input_data->pBody_RPY_des);
  Vec3<T> zero_vec3; 
  zero_vec3.setZero();

  _body_ori_task->UpdateTask(&_quat_des, _input_data->vBody_Ori_des, zero_vec3);
  _body_pos_task->UpdateTask(
      &(_input_data->pBody_des), 
      _input_data->vBody_des, 
      _input_data->aBody_des);
  _task_list.push_back(_body_ori_task);
  _task_list.push_back(_body_pos_task);
  for(size_t leg(0); leg<4; ++leg){
    if(_input_data->contact_state[leg] > 0.){ // Contact
      _foot_contact[leg]->setMu((T)_input_data->muv[leg]);
      _foot_contact[leg]->setRpla(_input_data->Rpla[leg]);//(Eigen::Matrix<T,3,3>)
      _foot_contact[leg]->setRFDesired((DVec<T>)(_input_data->Fr_des[leg]));
      _foot_contact[leg]->UpdateContactSpec();
      _contact_list.push_back(_foot_contact[leg]);
    }else{ // No Contact (swing)
      _foot_task[leg]->UpdateTask(
          &(_input_data->pFoot_des[leg]), 
          _input_data->vFoot_des[leg], 
          _input_data->aFoot_des[leg]);
          //zero_vec3);
      _task_list.push_back(_foot_task[leg]);
    }
  }
}


template<typename T>
void WBC_Ctrl<T>::_UpdateLegCMD(ControlFSMData<T> & data){
  LegControllerCommand<T> * cmd = data._legController->commands;
  //Vec4<T> contact = data._stateEstimator->getResult().contactEstimate;
  for (size_t leg(0); leg < cheetah::num_leg; ++leg) {
    cmd[leg].zero();
    for (size_t jidx(0); jidx < cheetah::num_leg_joint; ++jidx) {
      cmd[leg].tauFeedForward[jidx] = _tau_ff[cheetah::num_leg_joint * leg + jidx];
      cmd[leg].qDes[jidx] = _des_jpos[cheetah::num_leg_joint * leg + jidx];
      cmd[leg].qdDes[jidx] = _des_jvel[cheetah::num_leg_joint * leg + jidx];
      cmd[leg].kpJoint(jidx, jidx) = _Kp_joint[jidx];
      cmd[leg].kdJoint(jidx, jidx) = _Kd_joint[jidx];
    }
  }

  // Knee joint non flip barrier
  for(size_t leg(0); leg<4; ++leg){
    if(cmd[leg].qDes[2] < 0.3){
      cmd[leg].qDes[2] = 0.3;
    }
    if(data._legController->datas[leg].q[2] < 0.3){
      T knee_pos = data._legController->datas[leg].q[2]; 
      cmd[leg].tauFeedForward[2] = 1./(knee_pos * knee_pos + 0.02);
    }
  }
}


template<typename T>
void WBC_Ctrl<T>::_ParameterSetup(UserParameters* param){
  for(size_t i(0); i<3; ++i){
    ((BodyPosTask<T>*)_body_pos_task)->_Kp[i] = param->Kp_body[i];
    ((BodyPosTask<T>*)_body_pos_task)->_Kd[i] = param->Kd_body[i];
    ((BodyOriTask<T>*)_body_ori_task)->_Kp[i] = param->Kp_ori[i];
    ((BodyOriTask<T>*)_body_ori_task)->_Kd[i] = param->Kd_ori[i];
    for(size_t j(0); j<4; ++j){
      ((LinkPosTask<T>*)_foot_task[j])->_Kp[i] = param->Kp_foot[i];
      ((LinkPosTask<T>*)_foot_task[j])->_Kd[i] = param->Kd_foot[i];
    }
    _Kp_joint[i] = param->Kp_joint[i];
    _Kd_joint[i] = param->Kd_joint[i];
  }
}


template<typename T>
void WBC_Ctrl<T>::_CleanUp(){
  _contact_list.clear();
  _task_list.clear();
  //WBCtrl::_muv.clear();
}

template class WBC_Ctrl<float>;
template class WBC_Ctrl<double>;