#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "Kinematics.hpp"
#include "globals.hpp"


// class Kinematics;

using namespace Eigen;

class Controller
{
private:
    // Kinematics K;
    const double PI = M_PI;
    const double Ts = 0.001;
    
    Vector2d P_term[2] = {Vector2d::Zero(), Vector2d::Zero()};
    Vector2d I_term[2] = {Vector2d::Zero(), Vector2d::Zero()};
    Vector2d D_term[2] = {Vector2d::Zero(), Vector2d::Zero()};

    Vector2d P_term_old[2] = {Vector2d::Zero(), Vector2d::Zero()};
    Vector2d I_term_old[2] = {Vector2d::Zero(), Vector2d::Zero()};
    Vector2d D_term_old[2] = {Vector2d::Zero(), Vector2d::Zero()};

    Vector2d PID_output[2] = {Vector2d::Zero(), Vector2d::Zero()};
    
    Vector2d KP;
    Vector2d KI;
    Vector2d KD;
    
    Vector2d error[2] = {Vector2d::Zero(), Vector2d::Zero()};
    Vector2d error_old[2] = {Vector2d::Zero(), Vector2d::Zero()};
    
    Vector2d Ctrl_input[2] = {Vector2d::Zero(), Vector2d::Zero()};
    double cutoff_freq = 50;
    double tau = 1 / (2 * PI * cutoff_freq);
    
public:
    Controller(); // Kinematics &K
    ~Controller();
    Vector2d FB_controller(Vector2d error, Vector2d error_old, int Leg_num);
    Vector2d control();
    
};


#endif
