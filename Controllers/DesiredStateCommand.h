#ifndef DESIRED_STATE_COMMAND_H
#define DESIRED_STATE_COMMAND_H

#include <iostream>

struct DesiredStateCommand
{
    float x_vel_command;
    float y_vel_command;
    float yaw_vel_command;
    float roll_vel_command;

    DesiredStateCommand(float x_vel_command_default = 0.,
    float y_vel_command_default = 0.,
    float yaw_vel_command_default = 0.,
    float roll_vel_command_default = 0.)
    {
        float x_vel_command = x_vel_command_default;
        float y_vel_command = y_vel_command_default;
        float yaw_vel_command = yaw_vel_command_default;
        float roll_vel_command = roll_vel_command_default;
    }
};


#endif