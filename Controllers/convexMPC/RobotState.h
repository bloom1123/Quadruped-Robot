#ifndef _RobotState
#define _RobotState

#include "../../third-party/eigen3/Eigen/Dense"
#include "common_types.h"

#include "common_types.h"

using Eigen::Matrix;
using Eigen::Quaternionf;

class RobotState
{
    public:
        void set(flt* p, flt* v, flt* q, flt* w, flt* r, float mass);
        //void compute_rotations();
        void print();
        Matrix<fpt,3,1> p,v,w;
        Matrix<fpt,3,4> r_feet;
        Matrix<fpt,3,3> R;
        Matrix<fpt,3,3> I_body;
        Quaternionf q;
        fpt m = 20;
        //fpt m = 50.236; //DH
    //private:
};
#endif