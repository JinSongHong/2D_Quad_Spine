#ifndef ACTUATOR_H_
#define ACTUATOR_H_

#include "globals.hpp"

using namespace Eigen;

class filter;

class Actuator
{

private:
    filter F;
    int ACT_num;

    double d_cutoff = 70;
    double q = 0;
    double qd_tustin = 0;
    double qdd_tustin = 0;
    
    double q_old = 0;
    double qd_tustin_old = 0;
    double qdd_tustin_old = 0;    

    double q_bi = 0;
    double qd_bi_tustin = 0;
    double qdd_bi_tustin = 0;

    double q_bi_old = 0;
    double qd_bi_tustin_old = 0;
    double qdd_bi_tustin_old = 0;

public:
    Actuator(int n);
    ~Actuator();
    void Receive_data(const mjModel* m, mjData* d);
    double get_q(){return q;}
    double get_qd(){return qd_tustin;}
    double get_qdd(){return qdd_tustin;}
};

#endif