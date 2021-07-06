#ifndef HELP_FUNCTIONS_H_INCLUDED
#define HELP_FUNCTIONS_H_INCLUDED

#include <math.h>
#include <iostream>
#include <vector>

# define M_PI           3.14159265358979323846

using namespace std;

class Helpers {
public:

    constexpr static float STATIC_ONE_OVER_SQRT_2PI = 1 / sqrt(2 * M_PI);
    float ONE_OVER_SQRT_2PI = 1 / sqrt(2 * M_PI);

    static float normpdf(float x, float mu, float stdderiv) {
        /// x        --> observation measurement
        /// mu       --> pseudo range estimate
        /// stdderiv --> observation stdderiv

        return (STATIC_ONE_OVER_SQRT_2PI/stdderiv)*exp(-.5*pow((x-mu)/stdderiv, 2));
    }

};

#endif // HELP_FUNCTIONS_H_INCLUDED
