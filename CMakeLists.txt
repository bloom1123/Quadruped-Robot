cmake_minimum_required(VERSION 3.4)

project(quadruped_controller LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 14)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wa,-mbig-obj -O2")

add_subdirectory(third-party/pybind11)
include_directories("Utilities/")
include_directories("Math/")
include_directories("FSM_States")
include_directories("Dynamics")
include_directories("Controllers")
include_directories("Controllers/convexMPC")
include_directories("Controllers/WBC_Ctrl")
include_directories("third_party/eigen3")
include_directories("third-party/eigen3/include")
include_directories("third-party/qpOASES/include")


file(GLOB_RECURSE FSM_States_sources "FSM_States/*.cpp")

file(GLOB_RECURSE wbc_sources
"controllers/WBC_Ctrl/TaskSet/*.cpp"
"controllers/WBC_Ctrl/WBC/*.cpp"
"controllers/WBC_Ctrl/*.cpp"
)

file(GLOB_RECURSE robot_runner_sources "RobotRunner.cpp")
file(GLOB_RECURSE qp_sources_sources "third-party/qpOASES/src/*.cpp")
file(GLOB_RECURSE Goldfarb_Optimizer_sources "third-party/Goldfarb_Optimizer/*.cc")


pybind11_add_module(robot_controller
Dynamics/FloatingBaseModel.cpp
Dynamics/Quadruped.cpp
Controllers/LegController.cpp
Controllers/OrientationEstimator.cpp
Controllers/PositionVelocityEstimator.cpp
Controllers/convexMPC/Gait.cpp
Controllers/convexMPC/FootSwingTrajectory.cpp
Controllers/convexMPC/SolverMPC.cpp
Controllers/convexMPC/RobotState.cpp
Controllers/convexMPC/convexMPC_interface.cpp
Controllers/convexMPC/ConvexMPCLocomotion.cpp
${qp_sources_sources} 
${Goldfarb_Optimizer_sources}
${wbc_sources}
${FSM_States_sources}
${robot_runner_sources}
)

# add_executable(main
# ${qp_sources_sources} 
# ${Goldfarb_Optimizer_sources}
# ${controllers_sources}
# ${dynamics_sources}
# ${FSM_States_sources}
# ${robot_runner_sources}
# main.cpp)

# target_link_libraries(robot_controller PRIVATE pybind11::embed)
# target_link_libraries(main PRIVATE pybind11::embed)