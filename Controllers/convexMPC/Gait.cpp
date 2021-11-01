#include "Gait.h"
#include <iostream>

// Offset - Duration Gait
OffsetDurationGait::OffsetDurationGait(int nSegment, Vec4<int> offsets, Vec4<int> durations, const std::string &name) :
  _offsets(offsets.array()),
  _durations(durations.array()),
  _nIterations(nSegment){
  _name = name;
  // allocate memory for MPC gait table
  _mpc_table = new int[nSegment * 4];
  _offsetsFloat = offsets.cast<float>() / (float) nSegment;
  _durationsFloat = durations.cast<float>() / (float) nSegment;
  _stance = durations[0];
  _swing = nSegment - durations[0];
}

OffsetDurationGait::~OffsetDurationGait() {
  delete[] _mpc_table;
}

Vec4<float> OffsetDurationGait::getContactState(float vel_cmd) {

  Array4f offsets_temp,durations_temp;
  // if(vel_cmd < 0.1){
    // offsets_temp << 0., 0., 0., 0.;
    // durations_temp << 1., 1., 1., 1.;
  // }else{
    offsets_temp = _offsetsFloat;
    durations_temp = _durationsFloat;
  // }
  Array4f progress = _phase - offsets_temp;
  for(int i = 0; i < 4; i++){
    if(progress[i] < 0) progress[i] += 1.;
    if(progress[i] >= durations_temp[i]){
      progress[i] = 0.;
    }else{
      progress[i] = progress[i] / durations_temp[i];
    }
  }
  return progress.matrix();
}

Vec4<float> OffsetDurationGait::getSwingState(float vel_cmd)
{ 
  Array4f offsets_temp,durations_temp;
  // if(vel_cmd < 0.1){
  //   offsets_temp << 0., 0., 0., 0.;
  //   durations_temp << 1., 1., 1., 1.;
  // }else{
    offsets_temp = _offsetsFloat;
    durations_temp = _durationsFloat;
  // }

  Array4f swing_offset = offsets_temp + durations_temp;

  for(int i = 0; i < 4; i++){
    if(swing_offset[i] > 1){
      swing_offset[i] -= 1.;
    }
  }
  Array4f swing_duration = 1. - durations_temp;
  Array4f progress = _phase - swing_offset;
  for(int i = 0; i < 4; i++){
    if(progress[i] < 0){
      progress[i] += 1.f;
    }
    if(progress[i] >= swing_duration[i]){
      progress[i] = 0.;
    }else{
      progress[i] = progress[i] / swing_duration[i];
    }
  }
  return progress.matrix();
}

int* OffsetDurationGait::getMpcTable(){
  //printf("MPC table:\n");
  for(int i = 0; i < _nIterations; i++){
    int iter = (i + _iteration + 1) % _nIterations;
    Array4i progress = iter - _offsets;
    for(int j = 0; j < 4; j++){
      if(progress[j] < 0) progress[j] += _nIterations;
      if(progress[j] < _durations[j])
        _mpc_table[i*4 + j] = 1;
      else
        _mpc_table[i*4 + j] = 0;
    }
  }
  return _mpc_table;
}

void OffsetDurationGait::setIterations(int iterationsPerMPC, int currentIteration){
  _iteration = ((currentIteration / iterationsPerMPC) + gaitphascorr)% _nIterations;
  _phase = (float)((currentIteration + gaitphascorr*iterationsPerMPC)% (iterationsPerMPC * _nIterations)) / (float) (iterationsPerMPC * _nIterations);
  // std::cout << _phase << std::endl;
}

float OffsetDurationGait::getCurrentGaitPhase() {
  // return _iteration;
  return _phase;
}

float OffsetDurationGait::getCurrentSwingTime(float dtMPC, int leg) {//GET TOTAL SWING TIME
  (void)leg;
  return dtMPC * _swing;
}

float OffsetDurationGait::getCurrentStanceTime(float dtMPC, int leg) {
  (void) leg;//warning error
  return dtMPC * _stance;
}

int OffsetDurationGait::getHoriz() {
 return _nIterations;
}

MixedFrequncyGait::~MixedFrequncyGait() {
  delete[] _mpc_table;
}

Vec4<float> MixedFrequncyGait::getContactState(float vel_cmd) {
  Array4f progress = _phase;

  for(int i = 0; i < 4; i++) {
    if(progress[i] < 0) progress[i] += 1.;
    if(progress[i] > _duty_cycle) {
      progress[i] = 0.;
    } else {
      progress[i] = progress[i] / _duty_cycle;
    }
  }

  return progress.matrix();
}

Vec4<float> MixedFrequncyGait::getSwingState(float vel_cmd) {

  float swing_duration = 1.f - _duty_cycle;
  Array4f progress = _phase - _duty_cycle;
  for(int i = 0; i < 4; i++) {
    if(progress[i] < 0) {
      progress[i] = 0;
    } else {
      progress[i] = progress[i] / swing_duration;
    }
  }
  return progress.matrix();
}

MixedFrequncyGait::MixedFrequncyGait(int nSegment, Vec4<int> periods, float duty_cycle, const std::string &name) {
  _name = name;
  _duty_cycle = duty_cycle;
  _mpc_table = new int[nSegment * 4];
  _periods = periods;
  _nIterations = nSegment;
  _iteration = 0;
  _phase.setZero();
}

int* MixedFrequncyGait::getMpcTable() {
  //printf("MPC table (%d):\n", _iteration);
  for(int i = 0; i < _nIterations; i++) {
    for(int j = 0; j < 4; j++) {
      int progress = (i + _iteration + 1) % _periods[j];  // progress
      if(progress < (_periods[j] * _duty_cycle)) {
        _mpc_table[i*4 + j] = 1;
      } else {
        _mpc_table[i*4 + j] = 0;
      }
    }
  }
  return _mpc_table;
}

void MixedFrequncyGait::setIterations(int iterationsBetweenMPC, int currentIteration) {
  _iteration = (currentIteration / iterationsBetweenMPC);// % _nIterations;
  for(int i = 0; i < 4; i++) {
    int progress_mult = currentIteration % (iterationsBetweenMPC * _periods[i]);
    _phase[i] = ((float)progress_mult) / ((float) iterationsBetweenMPC * _periods[i]);
    //_phase[i] = (float)(currentIteration % (iterationsBetweenMPC * _periods[i])) / (float) (iterationsBetweenMPC * _periods[i]);
  }
  //printf("phase: %.3f %.3f %.3f %.3f\n", _phase[0], _phase[1], _phase[2], _phase[3]);
}

float MixedFrequncyGait::getCurrentGaitPhase() {
  return 0;
}

float MixedFrequncyGait::getCurrentSwingTime(float dtMPC, int leg) {
  return dtMPC * (1. - _duty_cycle) * _periods[leg];
}

float MixedFrequncyGait::getCurrentStanceTime(float dtMPC, int leg) {
  return dtMPC * _duty_cycle * _periods[leg];
}

int MixedFrequncyGait::getHoriz() {
 return _nIterations;
}