#include "Controller.hpp"

Controller::Controller()
{
    KP = {200, 200};
    KI = {0, 0};
    KD = {20,20};

}

Controller::~Controller(){}

Vector2d Controller::FB_controller(Vector2d error, Vector2d error_old, int i)
{   
    tau = 1 / (2 * PI * cutoff_freq);

    P_term[i] = KP.cwiseProduct(error);

    I_term[i] = KI.cwiseProduct(Ts / 2 * (error + error_old)) + I_term_old[i];
    
    D_term[i] = 2 * KD.cwiseProduct(1 / (2 * tau + Ts) * (error - error_old)) -
                       (Ts - 2 * tau) / (2 * tau + Ts) * D_term_old[i];

    PID_output[i] = P_term[i] + I_term[i] + D_term[i];
    
    // cout << i << ":  " << P_term[i][0] << "    " << P_term[i][1] << "   " << P_term_old[i][0] << "    " << P_term_old[i][1] << endl;


    P_term_old[i] = P_term[i];
    I_term_old[i] = I_term[i];
    D_term_old[i] = D_term[i];


    return PID_output[i];

}

Vector2d Controller::control()
{
    // Front Leg
    
    // FB_controller()

} 