#include "Kinematics.hpp"

Kinematics::Kinematics(int n, Actuator &ACT_FHIP, Actuator &ACT_FKNEE, Actuator &ACT_SPINE,
                    Actuator &ACT_RHIP, Actuator &ACT_RKNEE)
                    :ACT_FHIP(ACT_FHIP), ACT_FKNEE(ACT_FKNEE), ACT_SPINE(ACT_SPINE), ACT_RHIP(ACT_RHIP), ACT_RKNEE(ACT_RKNEE) 
                    
{
    Leg_num = n;
}

Kinematics::~Kinematics(){}

void Kinematics::Receive_data(const mjModel* m, mjData* d)
{
    ACT_FHIP.Receive_data(m,d);
    ACT_FKNEE.Receive_data(m,d);
    ACT_RHIP.Receive_data(m,d);
    ACT_RKNEE.Receive_data(m,d);
    ACT_SPINE.Receive_data(m,d);
}
void Kinematics::Cal_kinematics(double t)
{   


    
    // Biarticular

    F_q_bi[0] = ACT_FHIP.get_q();    
    F_q_bi[1] = ACT_FHIP.get_q() + ACT_FKNEE.get_q();

    R_q_bi[0] = ACT_RHIP.get_q();    
    R_q_bi[1] = ACT_RHIP.get_q() + ACT_RKNEE.get_q();


    //     cout << "Front Leg:   q1: " << ACT_FHIP.get_q() << "    q2: " << ACT_FKNEE.get_q() << endl;
    // cout << "Rear Leg:   q1: " << ACT_RHIP.get_q() << "    q2: " << ACT_RKNEE.get_q() << endl;
    for(int i = 0; i < 2; i ++)
    {
        F_qd_bi_tustin[i] = F.tustin_derivative(F_q_bi[i], F_q_bi_old[i], F_qd_bi_tustin_old[i], d_cutoff);
        F_qdd_bi_tustin[i] = F.tustin_derivative(F_qd_bi_tustin[i], F_qd_bi_tustin_old[i], F_qdd_bi_tustin_old[i], d_cutoff);
        R_qd_bi_tustin[i] = F.tustin_derivative(R_q_bi[i], R_q_bi_old[i], R_qd_bi_tustin_old[i], d_cutoff);
        R_qdd_bi_tustin[i] = F.tustin_derivative(R_qd_bi_tustin[i], R_qd_bi_tustin_old[i], R_qdd_bi_tustin_old[i], d_cutoff);
    }

    F_q_bi_old = F_q_bi;
    F_qd_bi_tustin_old = F_qd_bi_tustin;
    F_qdd_bi_tustin_old = F_qdd_bi_tustin;

    R_q_bi_old = R_q_bi;
    R_qd_bi_tustin_old = R_qd_bi_tustin;
    R_qdd_bi_tustin_old = R_qdd_bi_tustin;

    double F_q2 = F_q_bi[1] - F_q_bi[0];

    // pos[0] = 2 * L * cos((F_q_bi[1] - F_q_bi[0]) / 2); // r
    // pos[1] = (F_q_bi[0] + F_q_bi[1]) / 2; // theta_r

    // Jacb << L * sin(q2 / 2), - L * sin(q2 / 2),
    //         L * cos(q2/2), L * cos(q2/2);
    
    // Cartesian 
    
    F_q_bi[0] = ACT_FHIP.get_q();    
    F_q_bi[1] = ACT_FHIP.get_q() + ACT_FKNEE.get_q();
    
    R_q_bi[0] = ACT_RHIP.get_q();    
    R_q_bi[1] = ACT_RHIP.get_q() + ACT_RKNEE.get_q();

    pos[0][0] = -L*cos(F_q_bi[0]) - L*cos(F_q_bi[1]);
    pos[0][1] = L*sin(F_q_bi[0]) + L*sin(F_q_bi[1]);
    

    pos[1][0] = -L*cos(R_q_bi[0]) - L*cos(R_q_bi[1]);
    pos[1][1] = L*sin(R_q_bi[0]) + L*sin(R_q_bi[1]);
    
    // Jacb << -L *(sin(q[0]) - L * sin(q[0]+q[1])), -L * sin(q[0] + q[1]),
    //         L *(cos(q[0]) + L * cos(q[0]+q[1])), L * cos(q[0] + q[1]);

    F_Jacb << L *sin(F_q_bi[0]), L * sin(F_q_bi[1]),
            L *cos(F_q_bi[0]), L * cos(F_q_bi[1]);

    R_Jacb << L *sin(R_q_bi[0]), L * sin(R_q_bi[1]),
            L *cos(R_q_bi[0]), L * cos(R_q_bi[1]);

    // cout << F_Jacb << endl;

    vel[0] = F_Jacb * F_qd_bi_tustin;
    vel[1] = R_Jacb * R_qd_bi_tustin;
    
    // Cal_error(t);
}

