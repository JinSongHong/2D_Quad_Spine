#include "Actuator.hpp"

Actuator::Actuator(int n)
{
    ACT_num = n;
}

Actuator::~Actuator()
{}

void Actuator::Receive_data(const mjModel* m, mjData* d)
{
    
    q = d->qpos[ACT_num + 3];


    qd_tustin = F.tustin_derivative(q, q_old, qd_tustin_old, d_cutoff);
    qdd_tustin = F.tustin_derivative(qd_tustin, qd_tustin_old, qdd_tustin_old, d_cutoff);


    q_old = q;
    qd_tustin_old = qd_tustin;
    qdd_tustin_old = qdd_tustin;


    
}


