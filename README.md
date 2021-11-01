# Quadruped-Robot
基于mit-cheetah开源项目，结合了Derek-TH-Wang（pybullet仿真环境）和GabrielEGC（MPC动力学模型）所提出的方法。有以下功能：

- pybullet仿真
- 坡度行走
- 多机器人适配
- 消灭小碎步

- ......
- *还有更多待开发功能* 

# 编译
控制器底层仍是基于c++进行编写，利用pybind11提供python交互接口
```shell
mkdir build
cd build
cmake ..
make -j4
```
# 运行
使用pybullet仿真环境，运动控制器接受来自仿真环境的传感器数据，经过计算返回各关节控制力矩
```shell
python walking_simulation.py
```

# 机器人控制
控制器接口在RobotRunner.h中声明
```python
PYBIND11_MODULE(robot_controller, m) {
    py::class_<RobotRunner>(m, "robot_controller")
        .def(py::init<int>())
        .def("set_gait_type", &RobotRunner::setGaitType)        // 设置步态 4-stand 9-trot
        .def("set_control_mode", &RobotRunner::setRobotMode)    // 控制模式 1-PASSIVE 1-STAND_UP 2-LOCOMOTION
        .def("set_robot_vel", &RobotRunner::setRobotVel)        // 机器人速度 liner_x liner_y yaw_rate
        .def("set_shift_leg", &RobotRunner::setShiftLeg)        // 足端偏差（髋关节）
        .def("pre_work", &RobotRunner::PreWork)
        .def("run", &RobotRunner::run);
}
```

控制机器人需要使用游戏手柄，使用pygame进行数据交互，理论上可以支持各种不同类型的游戏手柄。

# 参考
Derek-TH-Wang /[quadruped_robot](https://github.com/Derek-TH-Wang/quadruped_ctrl)

GabrielEGC/[IHMC-Robotics](https://github.com/GabrielEGC/IHMC-Robotics/tree/master/MIT%20Mini-Cheetah)

mit-biomimetics/[Cheetah-Software](https://github.com/mit-biomimetics/Cheetah-Software)


