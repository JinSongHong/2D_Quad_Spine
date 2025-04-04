#ifndef Integrate_H_
#define Integrate_H_


#include "globals.hpp"

#include "Actuator.hpp"
#include "Kinematics.hpp"
#include "Trajectory.hpp"
#include "Controller.hpp"

class Actuator;
class Kinematics;
class Trajectory;
class Controller;

class Integrate
{
private:
    Kinematics& K;
    Trajectory& Traj;    
    Controller& C;
    
    double t;
    Vector2d pos[2] = {Vector2d::Zero(), Vector2d::Zero()};
    Vector2d ref_pos[2] = {Vector2d::Zero(), Vector2d::Zero()};
    Vector2d pos_err[2] = {Vector2d::Zero(), Vector2d::Zero()};
    Vector2d pos_err_old[2] = {Vector2d::Zero(), Vector2d::Zero()};

    Vector2d posFB_input[2] = {Vector2d::Zero(), Vector2d::Zero()};
    Vector2d Joint_input[2] = {Vector2d::Zero(), Vector2d::Zero()};

    
public:
    // Integrate(Actuator &A, Kinematics &K, Trajectory &Traj, Controller &C);
    Integrate(Kinematics &K, Trajectory &Traj, Controller &C);

    ~Integrate();
    void Receive_Data(const mjModel* m, mjData* d);
    void Cal_kinematics(double t);
    void Set_Trajectory(double t);
    void Cal_error();
    void Control();

    Vector2d F_Ctrl_input(){return Joint_input[0];}
    Vector2d R_Ctrl_input(){return Joint_input[1];}
    
    // Vector2d get_J_input(){return Joint_input;}
    // Vector2d get_ref_pos(){return ref_pos;}
    // Vector2d get_pos(){return pos;}
    
};


#endif