#ifndef FILTER_H_
#define FILTER_H_

#include "globals.hpp"

class filter
{
private:
    const double PI = M_PI;
    const double Ts = 0.001;


public:
    filter(/* args */);
    ~filter();
    double tustin_derivative(double input, double input_old, double output_old, double cutoff_freq);
};




#endif