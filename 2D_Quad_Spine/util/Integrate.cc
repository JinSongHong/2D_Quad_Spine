#include "Integrate.hpp"

Integrate::Integrate(Kinematics &K, Trajectory &Traj, Controller &C)
:K(K), Traj(Traj), C(C)
{
    
}

Integrate::~Integrate()
{
}


void Integrate::Receive_Data(const mjModel* m, mjData* d)
{
    K.Receive_data(m, d);
    
}

void Integrate::Cal_kinematics(double t)
{   
    K.Cal_kinematics(t);   
    
}

void Integrate::Set_Trajectory(double t)
{   
    
    ref_pos[0] = Traj.Traj_pos(t, 0); // Front
    ref_pos[1] = Traj.Traj_pos(t, 1); // Rear
    
    Cal_error();
}

void Integrate::Cal_error()
{
    for(int i = 0; i < 2; i++)
    {
        pos_err_old[i] = pos_err[i];
    }

    for(int i = 0; i < 2; i++)
    {
        pos[i] = K.get_pos(i);
        pos_err[i] = ref_pos[i] - pos[i]; 
    }

}

void Integrate::Control()
{   


    // cout << pos_err[0][0] << "    "  << pos_err[0][1] << "   " << pos_err[1][0] << "    "  << pos_err[1][1] << endl;

    for(int i = 0; i < 2; i++)
    posFB_input[i] = C.FB_controller(pos_err[i], pos_err_old[i], i);

    // cout << posFB_input[0] << endl;
    // posFB_input[0] = C.FB_controller(pos_err[0], pos_err_old[0]);

    // cout <<     pos_err[0][0] << "   " << pos_err_old[0][0] << endl; 
    // cout << "Cartesian:   " << posFB_input[0][0] << "   " << posFB_input[0][1] << endl;

    Joint_input[0] = K.get_F_JacbT() * posFB_input[0];
    Joint_input[1] = K.get_R_JacbT() * posFB_input[1];
    
    // cout << K.get_F_JacbT() << endl;

    // Biarticular
    for(int i = 0; i < 2; i++)
        Joint_input[i][0] += Joint_input[i][1]; 
    
    // Joint_input[0] = Joint_input[0] + Joint_input[1];
    // cout << "Input    :   taum: "<< Joint_input[0][0] << "   taub: " << Joint_input[0][1] << endl;


}
