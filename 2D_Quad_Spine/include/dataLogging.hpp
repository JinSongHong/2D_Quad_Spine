// #ifndef DATALOGGING_H_
// #define DATALOGGING_H_

// #include "Integrate.hpp"

// /***************** Data Logging *****************/
// FILE* fid;

// int loop_index = 0;
// const int data_frequency = 25; // frequency at which data is written to a file

// char datapath[] = "../data/data.csv";



// void init_save_data(FILE* fid)
// {
//     // This function is called once and is used to get the headers
//     // Write name of the variable here (header)
//     // comma(,) should be omitted in the last line.
    
//     fprintf(fid, "t,");
//     fprintf(fid, "q0,qd0,qdd0,"); 
//     fprintf(fid, "q1,qd1,qdd1,"); 
//     fprintf(fid, "joint1_input,joint2_input,");
//     fprintf(fid, "r_pos_ref,r_pos,th_pos_ref,th_pos");
//     // Don't remove the newline
//     fprintf(fid, "\n");
// }

// void save_data(const mjModel* m, mjData* d, Actuator &A , Integrate &I,FILE* fid)
// {

    
//     fprintf(fid, "%f, ", d->time);
//     for(int i = 0; i<2; i++)
//     {
//         fprintf(fid, "%f, %f, %f, ", A.get_q()[i], A.get_qd()[i], A.get_qdd()[i]);
//     }
//     fprintf(fid, "%f, %f, ", I.get_J_input()[0], I.get_J_input()[1]);
//     fprintf(fid, "%f, %f, %f, %f", I.get_ref_pos()[0], I.get_pos()[0], I.get_pos()[1], I.get_ref_pos()[1]);
    
//     // Don't remove the newline
//     fprintf(fid, "\n");


// }

// #endif // DATALOGGING_H_