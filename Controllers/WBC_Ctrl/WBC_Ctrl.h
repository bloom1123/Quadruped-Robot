#ifndef WBC_CONTROLLER_H
#define WBC_CONTROLLER_H

#include "../../FSM_States/ControlFSMData.h"
#include "../../Dynamics/FloatingBaseModel.h"
#include "../../Dynamics/Quadruped.h"
#include "../../cppTypes.h"
#include "WBC/WBIC.hpp"
#include "WBC/KinWBC.hpp"

template<typename T>
class LocomotionCtrlData{
  public:
    Vec3<T> pBody_des;
    Vec3<T> vBody_des;
    Vec3<T> aBody_des;
    Vec3<T> pBody_RPY_des;
    Vec3<T> vBody_Ori_des;

    Vec3<T> pFoot_des[4];
    Vec3<T> vFoot_des[4];
    Vec3<T> aFoot_des[4];
    Vec3<T> Fr_des[4];
    T muv[4];
    Eigen::Matrix<T,3,3> Rpla[4];

    Vec4<T> contact_state;

};


template<typename T>
class WBC_Ctrl{
  public:

    WBC_Ctrl(FloatingBaseModel<T> model);
    void run(void * input, ControlFSMData<T> & data);
  
  protected:

    void _UpdateModel(const StateEstimate<T> & state_est, const LegControllerData<T> * leg_data);
    void _ContactTaskUpdate(void * input, ControlFSMData<T> & data);
    void _UpdateLegCMD(ControlFSMData<T> & data);
    void _ComputeWBC();
    void _CleanUp();
    void _ParameterSetup(UserParameters* param);

    LocomotionCtrlData<T>* _input_data;
    Task<T>* _body_pos_task;
    Task<T>* _body_ori_task;
    Task<T>* _foot_task[4];
    ContactSpec<T>* _foot_contact[4];

    KinWBC<T>* _kin_wbc;
    WBIC<T>* _wbic;
    WBIC_ExtraData<T>* _wbic_data;

    FBModelState<T> _state;
    FloatingBaseModel<T> _model;

    DVec<T> _full_config;
    DVec<T> _tau_ff;
    DVec<T> _des_jpos;
    DVec<T> _des_jvel;

    std::vector<ContactSpec<T> * > _contact_list;
    std::vector<Task<T> * > _task_list;

    unsigned long long _iter;
    Quat<T> _quat_des;
    DMat<T> _A;
    DMat<T> _Ainv;
    DVec<T> _grav;
    DVec<T> _coriolis;

    Vec3<T> pre_foot_vel[4];
    Vec3<T> _Fr_result[4];
    std::vector<T> _Kp_joint, _Kd_joint;

};
#endif
