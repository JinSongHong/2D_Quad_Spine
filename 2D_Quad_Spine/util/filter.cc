#include "filter.hpp"

filter::filter(/* args */)
{
}

filter::~filter()
{
}

double filter::tustin_derivative(double input, double input_old, double output_old, double cutoff_freq)
{
    double time_const = 1 / (2 * PI * cutoff_freq);
    double output = 0;

    output = (2 * (input - input_old) - (Ts - 2 * time_const) * output_old) / (Ts + 2 * time_const);

    return output;
}
