#ifndef DESALINATOR_UTILS_H
#define DESALINATOR_UTILS_H

#include <Arduino.h>
#include <stdarg.h>

#define ROUND_TO_ZERO_IF_LOWER_THAN 0.08

bool containsNumber(String str);
float getMax(int numbers, ...);
float fuzzyAnd(float a, float b);
float fuzzyOr(float a, float b);
float fuzzyNot(float a);

float getNormalDistributionValue(float x, float mean, float standard_deviation);
float getZShapeDistributionValue(float x, float a, float b);
float getSShapeDistributionValue(float x, float a, float b);
float getTriangularShapeDistributionValue(float x, float a, float b);

#endif //DESALINATOR_UTILS_H
