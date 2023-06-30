#include "encoder.hpp"

double Encoder::get_reading_delta(double old_value, double new_value)
{
    // Take difference.
    double delta = new_value - old_value;
    // Apply modulus.
    delta = delta - Encoder::encoder_divisions * floor(delta / Encoder::encoder_divisions);
    if (delta > Encoder::encoder_divisions_over_2)
    {
        return -(Encoder::encoder_divisions - delta);
    }
    else
    {
        return delta;
    }
}
