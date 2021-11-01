#include <iostream>
#include "../../Math/orientation_tools.h"
#include "../../third-party/eigen3/Eigen/Dense"
#include "ConvexMPCLocomotion.h"
#include "convexMPC_interface.h"
#include "../../Utilities/pseudoInverse.h"

using Eigen::Matrix;
using Eigen::Array4i;
using Eigen::Dynamic;
using Eigen::Map;
using std::cout;
using std::endl;
//using Eigen::Quaternionf;
////////////////////
// Controller
////////////////////
ConvexMPCLocomotion::ConvexMPCLocomotion(float _dt, float gait_period_time) :
  horizonLength(10),
  dt(_dt),
  trotting(horizonLength, Vec4<int>(0,5,5,0), Vec4<int>(5,5,5,5),"Trotting"),
  bounding(horizonLength, Vec4<int>(0,0,5,5),Vec4<int>(5,5,5,5),"Bounding"),
  pronking(horizonLength, Vec4<int>(0,0,0,0),Vec4<int>(5,5,5,5),"Pronking"),
  galloping(horizonLength, Vec4<int>(0,2,7,9),Vec4<int>(5,5,5,5),"Galloping"),
  standing(horizonLength, Vec4<int>(0,0,0,0),Vec4<int>(10,10,10,10),"Standing"),
  trotRunning(horizonLength, Vec4<int>(0,5,5,0),Vec4<int>(4,4,4,4),"Trot Running"),
  walking(horizonLength, Vec4<int>(0,3,5,8), Vec4<int>(5,5,5,5), "Walking"),
  walking2(horizonLength, Vec4<int>(0,5,5,0), Vec4<int>(7,7,7,7), "Walking2"),
  pacing(horizonLength, Vec4<int>(5,0,5,0),Vec4<int>(5,5,5,5),"Pacing"),
  oneaerial(12, Vec4<int>(0,3,6,9), Vec4<int>(9,9,9,9), "Oneaerial"),
  random(horizonLength, Vec4<int>(9,13,13,9), 0.4, "Flying nine thirteenths trot"),
  random2(horizonLength, Vec4<int>(8,16,16,8), 0.5, "Double Trot")
{ 
  // iterationsBetweenMPC = int(gait_period_time/(dt*horizonLength));
  iterationsBetweenMPC = int(10);
  printf("perid: %.3f", gait_period_time);
  dtMPC = dt * iterationsBetweenMPC;
  printf("[Convex MPC] dt: %.3f iterations: %d, dtMPC: %.3f\n", dt, iterationsBetweenMPC, dtMPC);
  Rpla[0].setIdentity();
  Rpla[1].setIdentity();
  Rpla[2].setIdentity();
  Rpla[3].setIdentity();
  setup_problem(dtMPC, horizonLength, muv, 240, -1, Rpla);
  for(int i = 0; i < 4; i++){
    firstSwing[i] = true;
  }

  //use  for wbc 
  // pBody_des.setZero();
  // vBody_des.setZero();
  // aBody_des.setZero();
  Wpla.resize(4,3);
  Wpla.setZero();
  Wpla(0,0)=1;Wpla(1,0)=1;Wpla(2,0)=1;Wpla(3,0)=1;
  zfeet.setZero();
  apla.setZero();
  apla2(0)=-0.00357554;
  apla2(1)=0;
  apla2(2)=-0.269353;
  rpyplane.setZero();
  rpydesfin.setZero();

  // Vec3<float> norpla(-apla(1),-apla(2),1);
  // Vec3<float> nor2(apla(2),-apla(1),0);
  // norpla.normalize();
  // nor2.normalize();
  // Vec3<float> nor3;
  // nor3 = nor2.cross(norpla);
  // Rpla2[0].block(0,0,3,1)=nor3;
  // Rpla2[0].block(0,1,3,1)=nor2;
  // Rpla2[0].block(0,2,3,1)=norpla;

  rpyplane=rotationMatrixToRPY(Rpla[0]);
  rpyplane(2)=0;

  // Rpla2[0]=rpyToRotMat(rpyplane);
  // Rpla2[1]=rpyToRotMat(rpyplane);
  // Rpla2[2]=rpyToRotMat(rpyplane);
  // Rpla2[3]=rpyToRotMat(rpyplane);

}

void ConvexMPCLocomotion::initialize(){
  for(int i = 0; i < 4; i++) firstSwing[i] = true;
  firstRun = true;
}

void ConvexMPCLocomotion::recompute_timing(float gait_period_time) {
  iterationsBetweenMPC = int(gait_period_time/(dt*horizonLength));
  dtMPC = dt * iterationsBetweenMPC;
}

void ConvexMPCLocomotion::_SetupCommand(ControlFSMData<float> & data){
  
   _body_height = heightoffCoM + (float)data.userParameters->bo_height; // JUMPINGUNCOM
  
  
  float filterX(0.01), filterY(0.01), yaw_filter(0.005);

  yaw_vel_cmd = data._desiredStateCommand->yaw_vel_command;
  // _roll_turn_rate = data._desiredStateCommand->roll_vel_command;
  x_vel_cmd = data._desiredStateCommand->x_vel_command;
  y_vel_cmd = data._desiredStateCommand->y_vel_command;

  // x_vel_cmd = x_vel_cmd-fminf(fmaxf(x_vel_cmd,-0.1f),0.1f);
  // y_vel_cmd = y_vel_cmd-fminf(fmaxf(y_vel_cmd,-0.3f),0.3f);
  // _yaw_turn_rate = _yaw_turn_rate - fminf(fmaxf(_yaw_turn_rate,-0.1f),0.1f);
  // _roll_turn_rate = _roll_turn_rate - fminf(fmaxf(_roll_turn_rate,-0.1f),0.1f);

  // _yaw_turn_rate = (float)(data.userParameters->yawmaxfs)*_yaw_turn_rate;//
  // _roll_turn_rate = (float)(data.userParameters->rollmaxfs)*_roll_turn_rate;//
  
  _x_vel_des = _x_vel_des*(1-filterX) + (float)(data.userParameters->xmaxfs)*x_vel_cmd*filterX;//
  _y_vel_des = _y_vel_des*(1-filterY) + (float)(data.userParameters->ymaxfs)*y_vel_cmd*filterY;//
  _yaw_turn_rate = _yaw_turn_rate * (1 - yaw_filter) + (float)(data.userParameters->yawmaxfs) * yaw_vel_cmd * yaw_filter;

  // _yaw_des = data._stateEstimator->getResult().rpy[2] + dt * iterationsBetweenMPC * _yaw_turn_rate;
  //_roll_des = data._stateEstimator->getResult().rpy[0] + dt * iterationsBetweenMPC * _roll_turn_rate;
  //_yaw_des = fmaxf(fminf(_yaw_des + dt * _yaw_turn_rate,0.74f),-0.74f);
  //_roll_des = fmaxf(fminf(_roll_des + dt * _roll_turn_rate,0.74f),-0.74f);
  _yaw_des = _yaw_des + dt * _yaw_turn_rate;
  // _roll_des = _roll_des + dt * _roll_turn_rate;
  // cout << _x_vel_des << endl;
  // cout << _yaw_des << endl;
  _roll_des = 0.;
  _pitch_des = 0.;//*rpyplane(1);
}

template<>
void ConvexMPCLocomotion::run(ControlFSMData<float>& data) {
  bool omniMode = false;
  // Command Setup
  recompute_timing(data.userParameters->gait_period_time);//switch beetwen iBMPC and 27/2 for jumping...
  _SetupCommand(data);
  gaitNumber = data.userParameters->cmpc_gait;
  if(gaitNumber >= 10) {
    gaitNumber -= 10;
    omniMode = true;
  }

  if(stop_counter > 3){

    gaitNumber = 4;
  }

  auto& seResult = data._stateEstimator->getResult();
  // Check if transition to standing
  if(((gaitNumber == 4) && current_gait != 4) || firstRun){
    stand_traj[0] = seResult.position[0];
    stand_traj[1] = seResult.position[1];
    stand_traj[2] = data.userParameters->bo_height;
    stand_traj[3] = 0; // roll
    stand_traj[4] = 0; // pitch
    stand_traj[5] = seResult.rpy[2]; // yaw
    world_position_desired[0] = stand_traj[0];
    world_position_desired[1] = stand_traj[1];
  }
  previous_gait = current_gait;
  Gait* gait = &trotting;//9
  Gait* nxtgait = &trotting;//9
  if(gaitNumber == 1)
    gait = &bounding;
  else if(gaitNumber == 2)
    gait = &pronking;
  else if(gaitNumber == 3)
    gait = &walking2;
  else if(gaitNumber == 4)
    gait = &standing;
  else if(gaitNumber == 5)
    gait = &oneaerial;//trotRunning;
    //gait = &random2;
  else if(gaitNumber == 7)
    gait = &galloping;
  else if(gaitNumber == 8)
    gait = &pacing;

  // std::cout << "x_vel:" << _x_vel_des << std::endl;
  // std::cout << "y_vel:" << _y_vel_des << std::endl;
  
  nxtgait = gait;
  current_gait = gaitNumber;

  gait->setIterations(iterationsBetweenMPC, iterationCounter);
  nxtgait->setIterations(iterationsBetweenMPC, iterationCounter);
  // std::cout << gait->getCurrentGaitPhase() << std::endl;
  float vel_cmd = sqrt(x_vel_cmd*x_vel_cmd+y_vel_cmd*y_vel_cmd+yaw_vel_cmd*yaw_vel_cmd);
  float phase = gait->getCurrentGaitPhase();
  // std::cout << phase << std::endl;
  if(vel_cmd < 0.2){
    if(gait->getCurrentGaitPhase() == 0){
    
      // std::cout << "count " << stop_counter << std::endl;
      stop_counter += 1;
    }
  }else{
    stop_counter = 0;
  }

  // bool too_high = seResult.position[2] > 0.29;
  
  if(_body_height < 0.02) {
    _body_height = data.userParameters->bo_height;
  }
  if (current_gait!=previous_gait)//&& previous_gait!=6smooth transitions available
    bswitchgait = true;

  int* mpcTable = gait->getMpcTable();

  Vec3<float> v_des_robot(_x_vel_des, _y_vel_des, 0);
  Vec3<float> v_des_world = omniMode ? v_des_robot : seResult.rBody.transpose() * v_des_robot;

  for(int i = 0; i < 4; i++) {
    pFoot[i] = seResult.position + seResult.rBody.transpose() * (data._quadruped->getHipLocation(i) + data._legController->datas[i].p);
  }
  if(gait != &standing) {
    //world_position_desired += dt * Vec3<float>(v_des_world[0], v_des_world[1], 0);//MODIFY-CHECK RUNTIME
    world_position_desired =Vec3<float>(seResult.position[0], seResult.position[1], 0) + dt *iterationsBetweenMPC * Vec3<float>(v_des_world[0], v_des_world[1], 0);
  }
  // some first time initialization
  if(firstRun){
    world_position_desired[0] = seResult.position[0];
    world_position_desired[1] = seResult.position[1];
    world_position_desired[2] = seResult.rpy[2];
    for(int i = 0; i < 4; i++){
      footSwingTrajectories[i].setHeight(data.userParameters->swing_height);
      footSwingTrajectories[i].setInitialPosition(pFoot[i]);
      footSwingTrajectories[i].setFinalPosition(pFoot[i]);
    }
    firstRun = false;
  }
  // foot placement
  for(int l = 0; l < 4; l++)
    swingTimes[l] = gait->getCurrentSwingTime(dtMPC, l);
  float side_sign[4] = {-1, 1, -1, 1};
  float side_sign2[4] = {1, 1, -1, -1};
  float interleave_y[4] = {-0.08, 0.08, 0.02, -0.02};
  //float interleave_gain = -0.13;
  float interleave_gain = -0.2;
  //float v_abs = std::fabs(seResult.vBody[0]);
  float v_abs = std::fabs(v_des_robot[0]);
  for(int i = 0; i < 4; i++){
    if(firstSwing[i]) {
      swingTimeRemaining[i] = swingTimes[i];
    } else {
      swingTimeRemaining[i] -= dt;
    }
    //if(firstSwing[i]) {//footSwingTrajectories[i].setHeight(.05);
    
    // heightoffCoM_prev=heightoffCoM;
    
    float h = data.userParameters->swing_height+data.userParameters->swing_height*sqrt(apla(0)*apla(0)+apla(1)*apla(1));
    

    // std::cout << "vel:  " << vel << std::endl;
    // std::cout << "swing height: " << swing_height << std::endl;
    footSwingTrajectories[i].setHeight(h);//JUMPINGUNCOM
    
    Vec3<float> offset(side_sign2[i] * data.userParameters->shiftleg_x,
                       side_sign[i] * (data._quadruped->getAbadLength()+data.userParameters->shiftleg_y), 0);
    Vec3<float> pRobotFrame = (data._quadruped->getHipLocation(i) + offset);
    pRobotFrame[1] += interleave_y[i] * v_abs * interleave_gain;

    float stance_time = gait->getCurrentStanceTime(dtMPC, i); //for heuristics

    Vec3<float> pYawCorrected = coordinateRotation(CoordinateAxis::Z, -_yaw_turn_rate* stance_time / 2) * pRobotFrame;
    
    Vec3<float> des_vel;
    des_vel[0] = _x_vel_des;
    des_vel[1] = _y_vel_des;
    des_vel[2] = 0.0;

    Vec3<float> Pf = seResult.position + seResult.rBody.transpose() * (pYawCorrected + des_vel * swingTimeRemaining[i]);//this produces variations
    //+ seResult.vWorld * swingTimeRemaining[i];//float p_rel_max = 0.35f;
    float p_rel_max = 0.3f;//0.3f;
    // Using the estimated velocity is correct//Vec3<float> des_vel_world = seResult.rBody.transpose() * des_vel;

    float pfx_rel = seResult.vWorld[0] * (.5 + data.userParameters->cmpc_bonus_swing) * stance_time +
      .03f*(seResult.vWorld[0]-v_des_world[0]) +
      (0.5f*seResult.position[2]/9.81f) * (seResult.vWorld[1]*_yaw_turn_rate);

    float pfy_rel = seResult.vWorld[1] * .5 * stance_time +
      .03f*(seResult.vWorld[1]-v_des_world[1]) +
      (0.5f*seResult.position[2]/9.81f) * (-seResult.vWorld[0]*_yaw_turn_rate);

    // pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
    // pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);

    float pf_norm=sqrt(pfx_rel*pfx_rel+pfy_rel*pfy_rel);
    if (pf_norm>p_rel_max){
      pfx_rel=pfx_rel*p_rel_max/pf_norm;
      pfy_rel=pfy_rel*p_rel_max/pf_norm;
    }

    Pf[0] +=  pfx_rel;
    Pf[1] +=  pfy_rel;
    
    Pf[2] = apla(0)+apla(1)*Pf[0]+apla(2)*Pf[1];//JUMPINGUNC
    
    //Pf[2] = 0.0;
    footSwingTrajectories[i].setFinalPosition(Pf);
  }
  // calc gait// load LCM leg swing gains
  // Kp << 700, 0, 0,
  //    0, 700, 0,
  //    0, 0, 150;
  
  // Kd << 10, 0, 0,
  //    0, 10, 0,
  //    0, 0, 10;

  // Kp << 0, 0, 0,
  //    0, 0, 0,
  //    0, 0, 100.0;
  
  Kp = data.userParameters->kp_swing_mpc.asDiagonal();
  Kd = data.userParameters->kd_swing_mpc.asDiagonal();
  
  Kp_stance = data.userParameters->kp_stand_mpc.asDiagonal();
  Kd_stance = data.userParameters->kd_stand_mpc.asDiagonal();
  // gait
  Vec4<float> contactStates = gait->getContactState(vel_cmd);
  Vec4<float> swingStates = gait->getSwingState(vel_cmd);
  // std::cout << swingStates << std::endl;
  // std::cout << (iterationCounter % iterationsBetweenMPC) << std::endl;
  // std::cout << "------------" << std::endl;
  updateMPCIfNeeded(gait, mpcTable, data, omniMode, gait->getHoriz());

  Vec4<float> se_contactState(0,0,0,0);
  pestCoMx=0;
  pestCoMy=0;
  for(s16 foot = 0; foot < 4; foot++){
      if(swingStates[foot]==0){//SWING??
          Wpla(foot,1)=pFoot[foot](0);
          Wpla(foot,2)=pFoot[foot](1);
          zfeet(foot)=pFoot[foot](2);
          pseudoInverse(Wpla,0.001,Wplainv);
          apla=(1-(float)data.userParameters->coef_fil)*apla+((float)data.userParameters->coef_fil)*Wplainv*zfeet;
      }
    pestCoMx+=(pFoot[foot](0))/4;
    pestCoMy+=(pFoot[foot](1))/4;
  }
  
  if(data.userParameters->det_terrn>0.5){
    Vec3<float> norpla(-apla(1),-apla(2),1);
    Vec3<float> nor2(apla(2),-apla(1),0);
    norpla.normalize();
    nor2.normalize();
    Vec3<float> nor3;
    nor3 = nor2.cross(norpla);
    Rpla[0].block(0,0,3,1)=nor3;
    Rpla[0].block(0,1,3,1)=nor2;
    Rpla[0].block(0,2,3,1)=norpla;
 
    rpyplane=rotationMatrixToRPY(Rpla[0]);
    rpyplane(2)=0;

    Rpla[0]=rpyToRotMat(rpyplane);
    Rpla[1]=rpyToRotMat(rpyplane);
    Rpla[2]=rpyToRotMat(rpyplane);
    Rpla[3]=rpyToRotMat(rpyplane);
    heightoffCoM=apla(0)+apla(1)*pestCoMx+apla(2)*pestCoMy;
  }
  else{
    Matrix<float,3,1> RPYPlane1(0.8726,0,0);
    Matrix<float,3,1> RPYPlane2(-0.8726,0,0);
    Rpla[0]=rpyToRotMat(RPYPlane1);
    Rpla[1]=rpyToRotMat(RPYPlane2);
    Rpla[2]=rpyToRotMat(RPYPlane1);
    Rpla[3]=rpyToRotMat(RPYPlane2);

    heightoffCoM=0;
  }


  for(int foot = 0; foot < 4; foot++){
    float contactState = contactStates[foot];
    float swingState = swingStates[foot];
    if(swingState > 0){// foot is in swing
      if(firstSwing[foot]){
        firstSwing[foot] = false;
        footSwingTrajectories[foot].setInitialPosition(pFoot[foot]);
      }

      footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);

      Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
      Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
      Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - data._quadruped->getHipLocation(foot);
      Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);

      // Update for WBC
      pFoot_des[foot] = pDesFootWorld;
      vFoot_des[foot] = vDesFootWorld;
      aFoot_des[foot] = footSwingTrajectories[foot].getAcceleration();
      
      if(!data.userParameters->use_wbc){
        // Update leg control command regardless of the usage of WBIC
        data._legController->commands[foot].pDes = pDesLeg;
        data._legController->commands[foot].vDes = vDesLeg;
        data._legController->commands[foot].kpCartesian = Kp;
        data._legController->commands[foot].kdCartesian = Kd;
      }
      
    }else{ //foot is in stance
      firstSwing[foot] = true;
      Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
      Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
      Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - data._quadruped->getHipLocation(foot);
      Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);
      //cout << "Foot " << foot << " relative velocity desired: " << vDesLeg.transpose() << "\n";
      
      if(!data.userParameters->use_wbc){
        data._legController->commands[foot].pDes = pDesLeg;
        data._legController->commands[foot].vDes = vDesLeg;
        data._legController->commands[foot].kpCartesian = Kp_stance;
        data._legController->commands[foot].kdCartesian = Kd_stance;
        data._legController->commands[foot].forceFeedForward = f_ff[foot];
        data._legController->commands[foot].kdJoint = Mat3<float>::Identity() * 0.2;
      }else{ // Stance foot damping
        data._legController->commands[foot].pDes = pDesLeg;
        data._legController->commands[foot].vDes = vDesLeg;
        data._legController->commands[foot].kpCartesian = 0.*Kp_stance;
        data._legController->commands[foot].kdCartesian = Kd_stance;
      }
        
      //cout << "Foot " << foot << " force: " << f_ff[foot].transpose() << "\n";
      se_contactState[foot] = contactState;

    }
  }
  data._stateEstimator->setContactPhase(se_contactState);

  // Update For WBC
  // pBody_des[0] = world_position_desired[0];
  // pBody_des[1] = world_position_desired[1];
  // pBody_des[2] = _body_height;
  // vBody_des[0] = v_des_world[0];
  // vBody_des[1] = v_des_world[1];
  // vBody_des[2] = 0.;
  // aBody_des.setZero();
  // pBody_RPY_des[0] = rpydesfin(0);//_roll_des;
  // pBody_RPY_des[1] = rpydesfin(1); 
  // pBody_RPY_des[2] = rpydesfin(2);//_yaw_des;
  // vBody_Ori_des[0] = _roll_turn_rate;
  // vBody_Ori_des[1] = 0.;
  // vBody_Ori_des[2] = _yaw_turn_rate;
  // contact_state = gait->getContactState();// END of WBC Update

  iterationCounter++;
}


void ConvexMPCLocomotion::updateMPCIfNeeded(Gait* gait, int *mpcTable, ControlFSMData<float> &data, bool omniMode, int horLength) {
        
  int summpcT=0;
  for (int indmp=0; indmp < 4*horLength; indmp++) summpcT+=mpcTable[indmp];
  if (summpcT==0){
    apla=apla2;
  }
  //iterationsBetweenMPC = 30;
  if((iterationCounter % iterationsBetweenMPC) == 0){
    auto seResult = data._stateEstimator->getResult();
    Vec3<float> v_des_robot(_x_vel_des, _y_vel_des,0);
    Vec3<float> v_des_world = omniMode ? v_des_robot : seResult.rBody.transpose() * v_des_robot;
    Matrix<float,3,1> orienvecdes;

    if(current_gait == 4 || firstRun){
      orienvecdes <<_roll_des,_pitch_des,_yaw_des;
      Matrix<float,4,1> quatdes;
      if(data.userParameters->ori_pla_body<0.5){quatdes=rpyToQuat(orienvecdes);}
      else{quatdes=rotationMatrixToQuaternion(rpyToRotMat(orienvecdes)*Rpla[0].transpose());}
      Matrix<float,4,1> quatprev(seResult.orientation.data());
      if ((quatprev-quatdes).norm()>(quatprev+quatdes).norm()){quatdes=-quatdes;}
      rpydesfin=quatToRPY(quatdes);
      float trajInitial[13] = {quatdes(0), quatdes(1), quatdes(2), quatdes(3), // quat
        (float)stand_traj[0],(float)stand_traj[1],(float)_body_height, // pos
        0,0,0, // omega
        0,0,0}; // vel
      for(int i = 0; i < horLength; i++)
        for(int j = 0; j < 13; j++)
          trajAll[13*i+j] = trajInitial[j];
    }
    else if(bswitchgait){//1
      orienvecdes <<_roll_des,_pitch_des,_yaw_des;
      Matrix<float,4,1> quatdes;

      
      if(data.userParameters->ori_pla_body<0.5){
        quatdes=rpyToQuat(orienvecdes);
      }else{
        quatdes=rotationMatrixToQuaternion(rpyToRotMat(orienvecdes)*Rpla[0].transpose());
      }
      
      float xStart = world_position_desired[0];
      float yStart = world_position_desired[1];
      float zStart = seResult.position[2];
      float xFinal = xStart+horLength*dtMPC * v_des_world[0];
      float yFinal = yStart+horLength*dtMPC * v_des_world[1];
      Matrix<float,4,1> quatprev(seResult.orientation.data());
      if ((quatprev-quatdes).norm()>(quatprev+quatdes).norm()){quatdes=-quatdes;}
      rpydesfin=quatToRPY(quatdes);
      heightoffCoM=apla2(0)+apla2(1)*xFinal+apla2(2)*yFinal;
      //_body_height = heightoffCoM + (float)data.userParameters->bo_hei_jum; // JUMPINGUNCOM
      float _body_height_f=heightoffCoM+(float)data.userParameters->bo_hei_jum;
      float _body_height_fdot=(_body_height_f-zStart)/horLength;

      float trajInitial[13] = {quatdes(0),quatdes(1),quatdes(2),quatdes(3),
        xStart,yStart,zStart,
        0,0,0,
        v_des_world[0],v_des_world[1],_body_height_fdot/(dtMPC)};
      for(int i = 0; i < horLength; i++){
        for(int j = 0; j < 13; j++)
          trajAll[13*i+j] = trajInitial[j];
        if(i != 0){
          trajAll[13*i + 4] = trajAll[13 * (i - 1) + 4] + dtMPC * v_des_world[0];
          trajAll[13*i + 5] = trajAll[13 * (i - 1) + 5] + dtMPC * v_des_world[1];
          trajAll[13*i + 6] = trajAll[13 * (i - 1) + 6] + _body_height_fdot;//dtMPC * (apla2(1)*v_des_world[0]+apla2(2)*v_des_world[1])
        }
      }
      bswitchgait = false;
    }
   else {
      update_traj(trajAll);
      Matrix<float,4,1> quatdes;
      orienvecdes <<_roll_des,_pitch_des,_yaw_des + dtMPC * horLength * _yaw_turn_rate;
      
      if(data.userParameters->ori_pla_body<0.5){
        quatdes=rpyToQuat(orienvecdes);
      }else{
        quatdes=rotationMatrixToQuaternion(rpyToRotMat(orienvecdes)*Rpla[0].transpose());
      }
      
      
      Matrix<float,4,1> quatprev(trajAll[13*(horLength-1)+0],trajAll[13*(horLength-1)+1],trajAll[13*(horLength-1)+2],trajAll[13*(horLength-1)+3]);
      if ((quatprev-quatdes).norm()>(quatprev+quatdes).norm()){quatdes=-quatdes;}
      rpydesfin=quatToRPY(quatdes);

      float trajEnd[13] = {quatdes(0),quatdes(1),quatdes(2),quatdes(3),
        world_position_desired[0] + dtMPC * horLength * v_des_world[0],
        world_position_desired[1] + dtMPC * horLength * v_des_world[1],
        (float)_body_height + dtMPC * horLength * (apla(1)*v_des_world[0]+apla(2)*v_des_world[1]),//(float)_body_height,
        0,0,0,
        v_des_world[0],v_des_world[1],apla(1)*v_des_world[0]+apla(2)*v_des_world[1]};
      for(s16 itra = 0; itra < 13; itra++)
        trajAll[13*(horLength-1)+itra] = trajEnd[itra];//becuase kjoyst gives a roughr
      rpydesfin=quatToRPY(quatdes);
      }

    solveDenseMPC(mpcTable, data, horLength);

  }
}

void ConvexMPCLocomotion::solveDenseMPC(int *mpcTable, ControlFSMData<float> &data, int horLength) {
  auto seResult = data._stateEstimator->getResult();
  
  Matrix<double,13,1> QMatr = data.userParameters->Qmat;
  double* Q = QMatr.data();
  double* weights = Q;
  float alpha = 4e-5; // make setting eventually
  float* p = seResult.position.data();
  float* v = seResult.vWorld.data();
  float* w = seResult.omegaWorld.data();
  float* q = seResult.orientation.data();
  float r[12];
  for(int i = 0; i < 12; i++)
    r[i] = pFoot[i%4][i/4]  - seResult.position[i/4];
  //printf("current posistion: %3.f %.3f %.3f\n", p[0], p[1], p[2]);
  if(alpha > 1e-4) {
    std::cout << "Alpha was set too high (" << alpha << ") adjust to 1e-5\n";
    alpha = 1e-5;
  }
  Vec3<float> pxy_act(p[0], p[1], 0);
  Vec3<float> pxy_des(world_position_desired[0], world_position_desired[1], 0);
  //Vec3<float> pxy_err = pxy_act - pxy_des;
  Vec3<float> vxy(seResult.vWorld[0], seResult.vWorld[1], 0);
  int summpcT=0;
  for (int indmp=0; indmp < 4*horLength; indmp++) summpcT+=mpcTable[indmp];
  dtMPC = dt * iterationsBetweenMPC;
  muv[0]=(float)data.userParameters->muest;
  muv[1]=(float)data.userParameters->muest;
  muv[2]=(float)data.userParameters->muest;
  muv[3]=(float)data.userParameters->muest;
  setup_problem(dtMPC,horLength,muv,240,summpcT, Rpla);
  update_problem_data_floats(p,v,q,w,r,weights,trajAll,alpha,mpcTable, data.userParameters->mass);
  for(int leg = 0; leg < 4; leg++){
    Vec3<float> f;
    for(int axis = 0; axis < 3; axis++){
      f[axis] = get_solution(leg*3 + axis);
    }
    f_ff[leg] = -seResult.rBody * f;
    // Fr_des[leg] = f;// Update for WBC
  }
}