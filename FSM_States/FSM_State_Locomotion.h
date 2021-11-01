
#ifndef FSM_STATE_LOCOMOTION_H
#define FSM_STATE_LOCOMOTION_H

#include "FSM_State.h"
#include "../Controllers/convexMPC/ConvexMPCLocomotion.h"

// template<typename T> class WBC_Ctrl;
// template<typename T> class LocomotionCtrlData;


template <typename T>
class FSM_State_Locomotion: public FSM_State<T> 
{
private:
    
    bool locomotionSafe();
    int iter = 0;

    ConvexMPCLocomotion* cMPCOld;
    // WBC_Ctrl<T> * _wbc_ctrl;
    // LocomotionCtrlData<T> * _wbc_data;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    FSM_State_Locomotion(ControlFSMData<T>* _controlFSMData);
    
    void onEnter();

    void run();

    FSM_StateName checkTransition();

    TransitionData<T> transition();

    void onExit();
};


#endif  // FSM_STATE_LOCOMOTION_H