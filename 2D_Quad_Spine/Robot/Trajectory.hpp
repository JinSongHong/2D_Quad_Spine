#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include "globals.hpp"
#include "Kinematics.hpp"

class Kinematics;

class Trajectory
{
private:
    Kinematics& K;

    Vector2d ref_pos[2] = {Vector2d::Zero(), Vector2d::Zero()};  // 0: Front,   1: Rear
    Vector2d ref_vel[2] = {Vector2d::Zero(), Vector2d::Zero()};

    Vector2d pos_err[2] = {Vector2d::Zero(), Vector2d::Zero()};
    Vector2d vel_err[2] = {Vector2d::Zero(), Vector2d::Zero()};
    
    Vector2d pos_err_old[2] = {Vector2d::Zero(), Vector2d::Zero()};
    Vector2d vel_err_old[2] = {Vector2d::Zero(), Vector2d::Zero()};

    double t_norm;
    double T_total;
    double T;
    double rc;
    double v;
    double r0;
    double Period;
public:
    Trajectory(Kinematics &K);
    ~Trajectory();
    Vector2d Traj_pos(double t, int Leg_num);
};



#endif
