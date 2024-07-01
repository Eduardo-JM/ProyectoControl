#include "utils.h"

/**
 * @param str String to check
 * @return true if str has a digit in it, false otherwise
 */
bool containsNumber(String str) {
    for (unsigned int i = 0; i < str.length(); i++) {
        if (isDigit(str.charAt(i)))
            return true;
    }
    return false;
}

/**
 * @summary Returns the max number in the number sequence
 * @param numbers number of numbers to compare
 * @param ... numbers to compare
 * @return the maximum value found in the variadic
 */
float getMax(int numbers, ...) {
    va_list args;
    va_start(args, numbers);

    float max = -INFINITY;
    for (int i = 0; i < numbers; i++) {
        float value = va_arg(args, double);
        if (value > max) {
            max = value;
        }
    }
    va_end(args);
    return max;
}

/**
 * @param a number 1
 * @param b number 2
 * @return the maximum of the two
 */
float fuzzyOr(float a, float b) {
    return fmax(a, b);
}

/**
 * @param a number 1
 * @param b number 2
 * @return the minimum of the two
 */
float fuzzyAnd(float a, float b) {
    return fmin(a, b);
}

/**
 * @param a number 1
 * @return the complement of a
 */
float fuzzyNot(float a){
    return 1 - a;
}

/**
 * @param x value to found
 * @param mean x axis displacement
 * @return f(x) of the normal distribution described by mean
 * and the standard deviation of the class
 */
float getNormalDistributionValue(float x, float mean, float standard_deviation) {
    float r = pow(
            exp(1),
            -pow(x - mean, 2) / (2 * pow(standard_deviation, 2))
    );
    return r < ROUND_TO_ZERO_IF_LOWER_THAN ? 0 : r;
}

/**
 * @param x value to found
 * @param a lower limit
 * @param b upper limit
 * @return f(x) of the s shape function with 0 at a and 1 at b
 */
float getSShapeDistributionValue(float x, float a, float b) {
    if (x > b) return 1;
    if (x < a) return 0;
    float r = (x - a) / ((float) (b - a));
    return r < ROUND_TO_ZERO_IF_LOWER_THAN ? 0 : r;
}

/**
 * @param x value to found
 * @param a lower limit
 * @param b upper limit
 * @return f(x) of the z shape function with 1 at a and 0 at b
 */
float getZShapeDistributionValue(float x, float a, float b) {
    if (x > b) return 0;
    if (x < a) return 1;
    float r = (b - x) / ((float) (b - a));
    return r < ROUND_TO_ZERO_IF_LOWER_THAN ? 0 : r;
}

/**
 * @param x value to found
 * @param a lower limit
 * @param b upper limit
 * @return f(x) of the triangular shape function with 0 at both a and b and 1 at (b+a)/2
 */
float getTriangularShapeDistributionValue(float x, float a, float b) {
    if (x > b) return 0;
    if (x < a) return 0;
    return 1 - (2 * abs(x - (a + b) / 2))/((float) b - a);
}