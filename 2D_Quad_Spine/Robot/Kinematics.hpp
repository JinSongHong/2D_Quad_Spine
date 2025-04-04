#ifndef KINEMATICS_H_
#define KINEMATICS_H_

#include "Trajectory.hpp"
#include "globals.hpp"
#include "Actuator.hpp"
#include "filter.hpp"


class Actuator;
class Trajectory;
class filter;

class Kinematics
{
private:
    filter F;
   
    Actuator &ACT_FHIP;
    Actuator &ACT_FKNEE;
    Actuator &ACT_SPINE;
    Actuator &ACT_RHIP;
    Actuator &ACT_RKNEE;
    

    int Leg_num;
    double L = 0.25;
    double d_cutoff = 70;
    double t = 0;

    Matrix2d Jacb = Matrix2d::Identity();
    Matrix2d F_Jacb = Matrix2d::Identity();
    Matrix2d R_Jacb = Matrix2d::Identity();

    Vector2d F_q_bi = Vector2d::Zero();
    Vector2d F_qd_bi = Vector2d::Zero();
    Vector2d F_qdd_bi = Vector2d::Zero();

    Vector2d F_q_bi_old = Vector2d::Zero();
    Vector2d F_qd_bi_old = Vector2d::Zero();
    Vector2d F_qdd_bi_old = Vector2d::Zero();

    Vector2d R_q_bi = Vector2d::Zero();
    Vector2d R_qd_bi = Vector2d::Zero();
    Vector2d R_qdd_bi = Vector2d::Zero();

    Vector2d R_q_bi_old = Vector2d::Zero();
    Vector2d R_qd_bi_old = Vector2d::Zero();
    Vector2d R_qdd_bi_old = Vector2d::Zero();

    Vector2d F_qd_bi_tustin = Vector2d::Zero();
    Vector2d F_qdd_bi_tustin = Vector2d::Zero();

    Vector2d F_qd_bi_tustin_old = Vector2d::Zero();
    Vector2d F_qdd_bi_tustin_old = Vector2d::Zero();
    
    Vector2d R_qd_bi_tustin = Vector2d::Zero();
    Vector2d R_qdd_bi_tustin = Vector2d::Zero();

    Vector2d R_qd_bi_tustin_old = Vector2d::Zero();
    Vector2d R_qdd_bi_tustin_old = Vector2d::Zero();

    Vector2d pos[2] = {Vector2d::Zero(), Vector2d::Zero()}; // [Front,Rear] [x,z]
    Vector2d vel[2] = {Vector2d::Zero(), Vector2d::Zero()};

    
    double Hip_Iyy = 0.001375;
    double Knee_Iyy = 1.8739e-04;

    
public:
    Kinematics(int n, Actuator &ACT_FHIP, Actuator &ACT_FKNEE, Actuator &ACT_SPINE,
                    Actuator &ACT_RHIP, Actuator &ACT_RKNEE);
    ~Kinematics();
    void Receive_data(const mjModel* m, mjData* d);
    void Cal_kinematics(double t);
    Matrix2d get_F_JacbT(){return F_Jacb.transpose();}
    Matrix2d get_R_JacbT(){return R_Jacb.transpose();}
    
    Vector2d get_pos(int Leg_num){return pos[Leg_num];}
};

#endif