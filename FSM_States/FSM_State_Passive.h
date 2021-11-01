#ifndef FSM_STATE_PASSIVE_H
#define FSM_STATE_PASSIVE_H

#include "FSM_State.h"

template <typename T>
class FSM_State_Passive : public FSM_State<T> 
{
private:
    int iter = 0;
public:
    FSM_State_Passive(ControlFSMData<T>* _controlFSMData);

    void onEnter();

    void run();

    FSM_StateName checkTransition();

    TransitionData<T> transition();

    void onExit();

    TransitionData<T> testTransition();

};

#endif  // FSM_STATE_PASSIVE_H