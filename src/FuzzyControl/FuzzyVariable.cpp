#include "FuzzyVariable.h"

FuzzyVariable::FuzzyVariable(fuzzy_limits limits, int standard_deviation) {
    this->limits = limits;
    this->standard_deviation = standard_deviation;
}

FuzzyVariable::FuzzyVariable() {}

/**
 * Gets the degrees of membership at certain point
 * returns all fuzzy memberships
 * @param value
 * @return degrees of membership of the fuzzy variable
 */
fuzzy_var_membership FuzzyVariable::getMemberships(float value) {
    fuzzy_var_membership m;
    m.very_low = getZShapeDistributionValue(value, limits.very_low_a, limits.very_low_b);
    m.low = getNormalDistributionValue(value, limits.low_mean, standard_deviation);
    m.medium = getNormalDistributionValue(value, limits.medium_mean, standard_deviation);
    m.high = getNormalDistributionValue(value, limits.high_mean, standard_deviation);
    m.very_high = getSShapeDistributionValue(value, limits.very_high_a, limits.very_high_b);
    return m;
}