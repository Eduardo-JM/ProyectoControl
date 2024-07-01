#ifndef DESALINATOR_FUZZYCONTROLLER_H
#define DESALINATOR_FUZZYCONTROLLER_H

#include <Arduino.h>
#include "../utils.h"
#include "FuzzyVariable.h"
#include "EzoPmp.h"

/**
 * Defuzzification right limit
 */
#define MAMDANI_MAX_LIMIT 28
/**
 * @brief maximum temperature allowed before
 * pumping at max speed
 */
#define MAX_TEMPERATURE_ALLOWED 67.0

typedef struct output_fuzzy_limits {
    float very_low_a;
    float very_low_b;
    float low_a;
    float low_b;
    float medium_a;
    float medium_b;
    float high_a;
    float high_b;
    float very_high_a;
    float very_high_b;
    float extremely_high_a;
    float extremely_high_b;
} output_fuzzy_limits;

typedef struct fuzzy_output {
    bool active;
    float very_low;
    float low;
    float medium;
    float high;
    float very_high;
    float extremely_high;
} fuzzy_output;

class FuzzyController {
public:
    FuzzyController (EzoPmp pmp);
    void handlePumpFlow(float seawater_temperature,
                        float receiver_temperature,
                        float heated_water_temperature);
    float getFlowRate();
private:
    float defuzzify(float seawater_temperature, float receiver_temperature);
    fuzzy_output setFlowMembership(
            float seawater_temperature,
            float receiver_temperature);
    FuzzyVariable fuzzySeawater;
    FuzzyVariable fuzzyReceiver;
    output_fuzzy_limits output_limits;
    float flow_rate;
    EzoPmp pmp;
};

#endif //DESALINATOR_FUZZYCONTROLLER_H
