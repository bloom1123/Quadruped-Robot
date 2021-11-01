#ifndef FSM_STATE_STANDUP_H
#define FSM_STATE_STANDUP_H

#include "FSM_State.h"
#include <iostream>


template <typename T>
class FSM_State_StandUp : public FSM_State<T>
{
private:

  int iter = 0;
  std::vector<Vec3<T>> _ini_foot_pos;

public:
  FSM_State_StandUp(ControlFSMData<T>* _controlFSMData);

  void onEnter();
  
  void run();

  FSM_StateName checkTransition();

  TransitionData<T> transition();

  void onExit();

};

#endif  // FSM_STATE_STANDUP_H