// #include "RobotRunner.h"
#include "robot_settings.h"

int main()
{   
    std::vector<double> legdata = {0.0, -0.8, 1.6, 0.0, -0.8, 1.6, 0.0, -0.8, 1.6, 0.0, -0.8, 1.6};
    std::vector<double> imudata = {0.0, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
    std::vector<double> vel_command = {0, 0, 0, 0};

    // RobotRunner* robot_runner = new RobotRunner();

    ControlParameters* ctrlParams = new ControlParameters();
    std::cout << ctrlParams->controller_dt << std::endl;
    std::cout << typeid(ctrlParams->control_mode).name() << std::endl;
    std::cout << "--------------" << std::endl;
    
    // UserParameters userParams;
    // std::cout << userParams.Kd_joint<< std::endl;
    // std::cout << userParams.muest<< std::endl;
    
    // for(int i=0;i<=100;i++){
    //     std::vector<double> effort = robot_runner->run(imudata, legdata, vel_command);
    //     std::cout << "run times : " << i << std::endl;
    // }
    return 0;
}