#ifndef DESALINATOR_FUZZY_VARIABLE_H
#define DESALINATOR_FUZZY_VARIABLE_H

#include <Arduino.h>
#include "../utils.h"

/**
 * @brief Standard deviations of fuzzy
 * normal distribution shapes
 */
enum DISTRIBUTION_SIGMA {
    SEAWATER_STANDARD_DEVIATION = 3,
    SOLAR_RECEIVER_STANDARD_DEVIATION = 8
};

typedef struct fuzzy_var_membership {
    float very_low;
    float low;
    float medium;
    float high;
    float very_high;
} fuzzy_var_membership;

typedef struct fuzzy_limits {
    float very_low_a;
    float very_low_b;
    float low_mean;
    float medium_mean;
    float high_mean;
    float very_high_a;
    float very_high_b;
} fuzzy_limits;

class FuzzyVariable {
public:
    FuzzyVariable();
    FuzzyVariable(fuzzy_limits limits, int standard_deviation);
    fuzzy_var_membership getMemberships(float value);
private:
    fuzzy_limits limits;
    int standard_deviation;
};


#endif //DESALINATOR_FUZZY_VARIABLE_H
