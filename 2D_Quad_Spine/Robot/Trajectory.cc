#include "Trajectory.hpp"

Trajectory::Trajectory(Kinematics &K)
:K(K)
{
    T_total = 1;
    T = T_total/2;
    r0 = 0.3536;
    rc = 0.1;
    v = 0.;
    Period = 30;
}

Trajectory::~Trajectory(){}

Vector2d Trajectory::Traj_pos(double t, int Leg_num)
{
    double T = 500000;
    
    switch(Leg_num) {

        case 0:  // Front
            if(t < T)
            {
                ref_pos[Leg_num][0] = 0; //0.3*sin(t); //0.2*sin(t); //sin(t);
                ref_pos[Leg_num][1] = 0.1*sin(t) + 0.3536; // 
            }
            else
            {
                // gait_pos(t-K);
            }

            break;

        case 1: // Rear
            if(t < T)
            {
                ref_pos[Leg_num][0] = 0;//0.3*sin(t); //0.2*sin(t); //sin(t);
                ref_pos[Leg_num][1] = 0.1*sin(t) + 0.3536;
            }
            else
            {
                // gait_pos(t-K);
            }
            
            break;
    }
    

    return ref_pos[Leg_num];
}

