#ifndef PROJECT_GAIT_H
#define PROJECT_GAIT_H

#include <string>
#include <queue>

#include "../../cppTypes.h"


class Gait {
public:
  virtual ~Gait() = default;

  virtual Vec4<float> getContactState(float vel_cmd) = 0;
  virtual Vec4<float> getSwingState(float vel_cmd) = 0;
  virtual int* getMpcTable() = 0;
  virtual void setIterations(int iterationsBetweenMPC, int currentIteration) = 0;
  virtual float getCurrentStanceTime(float dtMPC, int leg) = 0;
  virtual float getCurrentSwingTime(float dtMPC, int leg) = 0;
  virtual float getCurrentGaitPhase() = 0;
  virtual int getHoriz() = 0;
  int gaitphascorr = 0; 

protected:
  std::string _name;
};

using Eigen::Array4f;
using Eigen::Array4i;

class OffsetDurationGait : public Gait {
public:
  OffsetDurationGait(){}
  OffsetDurationGait(int nSegment, Vec4<int> offset, Vec4<int> durations, const std::string& name);
  ~OffsetDurationGait();
  Vec4<float> getContactState(float vel_cmd);
  Vec4<float> getSwingState(float vel_cmd);
  int* getMpcTable();
  void setIterations(int iterationsBetweenMPC, int currentIteration);
  float getCurrentStanceTime(float dtMPC, int leg);
  float getCurrentSwingTime(float dtMPC, int leg);
  float getCurrentGaitPhase();
  int getHoriz();
  Array4i _offsets; // offset in mpc segments
  Array4i _durations; // duration of step in mpc segments
  Array4f _offsetsFloat; // offsets in phase (0 to 1)
  Array4f _durationsFloat; // durations in phase (0 to 1)
  int _nIterations;
  int _iteration;
  float _phase;
  
private:
  int* _mpc_table;
  int _stance;
  int _swing;
  
};



class MixedFrequncyGait : public Gait {
public:
  MixedFrequncyGait(int nSegment, Vec4<int> periods, float duty_cycle, const std::string& name);
  ~MixedFrequncyGait();
  Vec4<float> getContactState(float vel_cmd);
  Vec4<float> getSwingState(float vel_cmd);
  int* getMpcTable();
  void setIterations(int iterationsBetweenMPC, int currentIteration);
  float getCurrentStanceTime(float dtMPC, int leg);
  float getCurrentSwingTime(float dtMPC, int leg);
  float getCurrentGaitPhase();
  int getHoriz();

private:
  float _duty_cycle;
  int* _mpc_table;
  Array4i _periods;
  Array4f _phase;
  int _iteration;
  int _nIterations;
};

#endif //PROJECT_GAIT_H
