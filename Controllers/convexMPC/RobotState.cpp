#include "RobotState.h"
#include "../../third-party/eigen3/Eigen/Dense"
#include <iostream>
#include <math.h>

using std::cout;
using std::endl;

void RobotState::set(flt* p_, flt* v_, flt* q_, flt* w_, flt* r_, float mass)
{
    for(u8 i = 0; i < 3; i++)
    {
        this->p(i) = p_[i];
        this->v(i) = v_[i];
        this->w(i) = w_[i];
    }
    this->q.w() = q_[0];
    this->q.x() = q_[1];
    this->q.y() = q_[2];
    this->q.z() = q_[3];

    //for(u8 i = 0; i < 12; i++)
    //    this->r_feet(i) = r[i];
    for(u8 rs = 0; rs < 3; rs++)
        for(u8 c = 0; c < 4; c++)
            this->r_feet(rs,c) = r_[rs*4 + c];
        
    R = this->q.toRotationMatrix();
    Matrix<fpt,3,1> Id;
    Id << .07f, 0.26f, 0.242f;
    // Id << 0.3f, 2.1f, 2.1f; // DH
    this->m = mass;
    I_body.diagonal() = Id;
}







