
#ifndef PROJECT_ROBOTRUNNER_H
#define PROJECT_ROBOTRUNNER_H

#include "cppTypes.h"
#include "FSM_States/ControlFSM.h"
#include "Controllers/StateEstimatorContainer.h"
#include "Utilities/IMUTypes.h"
#include "Controllers/ContactEstimator.h"
#include "Controllers/DesiredStateCommand.h"
#include "Controllers/LegController.h"
#include "Dynamics/Quadruped.h"
#include "Controllers/RobotLegState.h"
#include <iostream>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
// #include <pybind11/eigen.h>


namespace py=pybind11;

class RobotRunner{
 public:
  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RobotRunner(int name);
  // void init();
  std::vector<double> run(std::vector<double> imuData,
           std::vector<double> motorData);
  void cleanup();

  void initializeStateEstimator();
  void setRobotVel(std::vector<double> vel);
  void setRobotMode(int);
  void setGaitType(int);
  void setShiftLeg(std::vector<double> vec);
  void PreWork(std::vector<double> imuData, std::vector<double> motorData);
  virtual ~RobotRunner();

  RobotType robotType;
  VectorNavData vectorNavData;

  DesiredStateCommand* desiredStateCommand;
  ControlParameters* controlParameters;
  UserParameters* userParameters;
  LegData legdata;
  LegCommand legcommand;

 private:

  int iter = 0;
  ControlFSM<float>* _controlFSM;
  Quadruped<float> _quadruped;
  LegController<float>* _legController;
  StateEstimate<float> _stateEstimate;
  StateEstimatorContainer<float>* _stateEstimator;
  

  u64 _iterations = 0;
};

PYBIND11_MODULE(robot_controller, m) {
    py::class_<RobotRunner>(m, "robot_controller")
        .def(py::init<int>())
        .def("set_gait_type", &RobotRunner::setGaitType)
        .def("set_control_mode", &RobotRunner::setRobotMode)
        .def("set_robot_vel", &RobotRunner::setRobotVel)
        .def("set_shift_leg", &RobotRunner::setShiftLeg)
        .def("pre_work", &RobotRunner::PreWork)
        .def("run", &RobotRunner::run);
}

#endif  // PROJECT_ROBOTRUNNER_H
