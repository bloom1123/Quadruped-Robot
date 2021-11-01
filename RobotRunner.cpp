#include <iostream>
#include "RobotRunner.h"
#include "Controllers/ContactEstimator.h"
#include "Controllers/OrientationEstimator.h"
#include "Dynamics/MiniCheetah.h"
#include "Dynamics/A1.h"
#include "Dynamics/Spot.h"
#include "Dynamics/Anymal_b.h"
#include "Controllers/PositionVelocityEstimator.h"

RobotRunner::RobotRunner(int name)
{ 

  desiredStateCommand = new DesiredStateCommand();
  controlParameters = new ControlParameters();
  userParameters = new UserParameters();


  switch (name)
  {
  case 0:
    _quadruped = buildMiniCheetah<float>(userParameters);
    break;

  case 1:
    _quadruped = buildA1<float>();
    userParameters->Qmat << 1.0, 1.0, 1.0, 1.0, 3.0, 3.0, 10.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.2;
    
    userParameters->kp_stand_up << 900, 900, 900;
    userParameters->kd_stand_up << 15, 15, 15;

    userParameters->kp_swing_mpc << 400, 400, 150;
    userParameters->kd_swing_mpc << 12, 12, 12;

    userParameters->kp_stand_mpc << 0, 0, 0;
    userParameters->kd_stand_mpc << 15, 15, 15;
    break;

  case 2:
    _quadruped = buildSpot<float>();
    
    userParameters->bo_height = 0.45;
    userParameters->bo_hei_jum = 0.5;
    userParameters->swing_height = 0.15;
    userParameters->mass = 20;
    userParameters->xmaxfs = 1.3;
    userParameters->yawmaxfs = 1.0;
    userParameters->ymaxfs = 1.0;
    
    userParameters->kp_stand_up << 900, 900, 900;
    userParameters->kd_stand_up << 15, 15, 15;

    userParameters->kp_swing_mpc << 500, 500, 500;
    userParameters->kd_swing_mpc << 20, 20, 20;

    userParameters->kp_stand_mpc << 0, 0, 0;
    userParameters->kd_stand_mpc << 15, 15, 15;

    userParameters->Qmat << 1.0, 1.0, 1.0, 1.0, 3.0, 3.0, 10.0, 0., 0., 0.2, 0.2, 0.2, 0.2;
    break;

  case 3:
    _quadruped = buildANYMAY_B<float>(userParameters);
    
    break;

  default:
    _quadruped = buildMiniCheetah<float>(userParameters);
    break;
  }
  

  // Build the appropriate Quadruped object
  std::cout << "create quadruped" << std::endl;

  // Always initialize the leg controller and state entimator
  _legController = new LegController<float>(_quadruped);

  // std::cout << "init legcontroller" << std::endl;
  _stateEstimator = new StateEstimatorContainer<float>(
      &vectorNavData, 
      _legController->datas,
      &_stateEstimate, 
      controlParameters);

  std::cout << "init stateEstimator" << std::endl;

  initializeStateEstimator();

  _controlFSM = new ControlFSM<float>(&_quadruped, 
                                      _legController,
                                      _stateEstimator,
                                      desiredStateCommand, 
                                      controlParameters, 
                                      userParameters);
  std::cout << "init finish" << std::endl;
}


void RobotRunner::PreWork(std::vector<double> imuData, std::vector<double> motorData){
  vectorNavData.accelerometer(0, 0) = imuData[0];
  vectorNavData.accelerometer(1, 0) = imuData[1];
  vectorNavData.accelerometer(2, 0) = imuData[2];
  vectorNavData.quat(0, 0) = imuData[3];
  vectorNavData.quat(1, 0) = imuData[4];
  vectorNavData.quat(2, 0) = imuData[5];
  vectorNavData.quat(3, 0) = imuData[6];
  vectorNavData.gyro(0, 0) = imuData[7];
  vectorNavData.gyro(1, 0) = imuData[8];
  vectorNavData.gyro(2, 0) = imuData[9];
  
  // update leg data
  for (int i = 0; i < 4; i++) {
    legdata.q_abad[i] = motorData[i * 3];
    legdata.q_hip[i] = motorData[i * 3 + 1];
    legdata.q_knee[i] = motorData[i * 3 + 2];
    legdata.qd_abad[i] = motorData[12 + i * 3];
    legdata.qd_hip[i] = motorData[12 + i * 3 + 1];
    legdata.qd_knee[i] = motorData[12 + i * 3 + 2];
  }
  _stateEstimator->run();
  _legController->updateData(&legdata);

}
std::vector<double> RobotRunner::run(std::vector<double> imuData,
                      std::vector<double> motorData) {
  // update imu data (ax ay az qx qy qz qw wx wy wz) 
  vectorNavData.accelerometer(0, 0) = imuData[0];
  vectorNavData.accelerometer(1, 0) = imuData[1];
  vectorNavData.accelerometer(2, 0) = imuData[2];
  vectorNavData.quat(0, 0) = imuData[3];
  vectorNavData.quat(1, 0) = imuData[4];
  vectorNavData.quat(2, 0) = imuData[5];
  vectorNavData.quat(3, 0) = imuData[6];
  vectorNavData.gyro(0, 0) = imuData[7];
  vectorNavData.gyro(1, 0) = imuData[8];
  vectorNavData.gyro(2, 0) = imuData[9];
  
  // update leg data
  for (int i = 0; i < 4; i++) {
    legdata.q_abad[i] = motorData[i * 3];
    legdata.q_hip[i] = motorData[i * 3 + 1];
    legdata.q_knee[i] = motorData[i * 3 + 2];
    legdata.qd_abad[i] = motorData[12 + i * 3];
    legdata.qd_hip[i] = motorData[12 + i * 3 + 1];
    legdata.qd_knee[i] = motorData[12 + i * 3 + 2];
  }

  // Run the state estimator step
  _stateEstimator->run();

  // Update the leg data
  _legController->updateData(&legdata);

  // Setup the leg controller for a new iteration
  _legController->zeroCommand();
  _legController->setEnabled(true);
  _legController->setMaxTorqueCheetah3(208.5); 
  // Run Control 
  _controlFSM->runFSM();

  // Sets the leg controller commands for the robot appropriate commands
  _legController->updateCommand(&legcommand);
  _iterations++;

  std::vector<double> effort(24);
  for (int i = 0; i < 4; i++) {
      effort[i * 3] = legcommand.tau_abad_ff[i];
      effort[i * 3 + 1] = legcommand.tau_hip_ff[i];
      effort[i * 3 + 2] = legcommand.tau_knee_ff[i];

      effort[i * 3 + 12] = legcommand.tau_abad_ori[i];
      effort[i * 3 + 13] = legcommand.tau_hip_ori[i];
      effort[i * 3 + 14] = legcommand.tau_knee_ori[i];
  }

  return effort;
}


void RobotRunner::setRobotVel(std::vector<double> vel){
  desiredStateCommand->x_vel_command = vel[0];
  desiredStateCommand->y_vel_command = vel[1];
  desiredStateCommand->yaw_vel_command = vel[2];
  desiredStateCommand->roll_vel_command = vel[3];
}


void RobotRunner::setRobotMode(int mode){
  controlParameters->control_mode = mode;
}

void RobotRunner::setGaitType(int gait_type){
  userParameters->cmpc_gait = gait_type;
}

void RobotRunner::setShiftLeg(std::vector<double> vec){
  userParameters->shiftleg_x = vec[0];
  userParameters->shiftleg_y = vec[1];
}

void RobotRunner::initializeStateEstimator() {
  _stateEstimator->removeAllEstimators();
  _stateEstimator->addEstimator<ContactEstimator<float>>();
  Vec4<float> contactDefault;
  contactDefault << 0.5, 0.5, 0.5, 0.5;
  _stateEstimator->setContactPhase(contactDefault);
  _stateEstimator->addEstimator<VectorNavOrientationEstimator<float>>();
  _stateEstimator->addEstimator<LinearKFPositionVelocityEstimator<float>>();
}

RobotRunner::~RobotRunner() {
  delete _legController;
  delete _stateEstimator;
}

void RobotRunner::cleanup() {}
