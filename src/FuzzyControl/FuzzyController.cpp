#include "FuzzyController.h"

FuzzyController::FuzzyController(EzoPmp pmp) {
    fuzzy_limits seawater_limits = {
            6,
            12,
            13,
            20,
            27,
            28,
            34
    };

    fuzzy_limits solar_receiver_limits = {
            40, 60,
            60,
            80,
            100,
            100,
            120
    };
    FuzzyVariable fuzzySeawater(seawater_limits, SEAWATER_STANDARD_DEVIATION);
    FuzzyVariable fuzzyReceiver(solar_receiver_limits, SOLAR_RECEIVER_STANDARD_DEVIATION);

    this->fuzzySeawater = fuzzySeawater;
    this->fuzzyReceiver = fuzzyReceiver;
    this->flow_rate = 0;
    this->pmp = pmp;
    this->output_limits = {
            5,
            9,
            6,
            12,
            9,
            15,
            12,
            18,
            15,
            21,
            18,
            22
    };
}

/**
 * Using fuzzy inference rules defined by an expert,
 * sets the fuzzy flow rate membership
 * @param seawater_temperature
 * @param receiver_temperature
 * @return fuzzy flow rate
 */
fuzzy_output FuzzyController::setFlowMembership(float seawater_temperature, float receiver_temperature) {
    fuzzy_output output;

    fuzzy_var_membership seawater_membership = fuzzySeawater.getMemberships(seawater_temperature);
    fuzzy_var_membership receiver_membership = fuzzyReceiver.getMemberships(receiver_temperature);

    if (receiver_membership.very_low > 0) {
        output.active = false;
        return output;
    }

    output.active = true;

    if(receiver_membership.low > 0) {
        if (seawater_membership.very_high > 0)
            output.medium = fmax(
                    output.medium,
                    fuzzyAnd(
                        receiver_membership.low,
                        seawater_membership.very_high
                    )
            );
        if (seawater_membership.high > 0)
            output.low = fmax(
                    output.low,
                    fuzzyAnd(
                        receiver_membership.low,
                        seawater_membership.high
                    )
            );
        if (seawater_membership.medium > 0)
            output.very_low = fmax(
                    output.very_low,
                    fuzzyAnd(
                        receiver_membership.low,
                        seawater_membership.medium
                    )
            );
        if (seawater_membership.low > 0 || seawater_membership.very_low > 0) {
            output.active = false;
            return output;
        }
    }

    if(receiver_membership.medium > 0) {
        if (seawater_membership.very_high > 0 || seawater_membership.high > 0)
            output.high = fmax(
                    output.high,
                    fuzzyAnd(
                        receiver_membership.medium,
                        fuzzyOr(
                                seawater_membership.very_high,
                                seawater_membership.high
                        )
                    )
            );
        if (seawater_membership.medium > 0)
            output.medium = fmax(
                    output.medium,
                    fuzzyAnd(
                        receiver_membership.medium,
                        seawater_membership.medium
                    )
            );

        if (seawater_membership.low > 0)
            output.low = fmax(
                    output.low,
                    fuzzyAnd(
                        receiver_membership.medium,
                        seawater_membership.low
                    )
            );
        if (seawater_membership.very_low > 0)
            output.very_low = fmax(
                    output.very_low,
                    fuzzyAnd(
                        receiver_membership.medium,
                        seawater_membership.very_low
                    )
            );
    }

    if(receiver_membership.high > 0) {
        if (seawater_membership.very_high > 0 || seawater_membership.high > 0)
            output.very_high = fmax(
                    output.very_high,
                    fuzzyAnd(
                        receiver_membership.high,
                        fuzzyOr(
                                seawater_membership.very_high,
                                seawater_membership.high
                        )
                    )
            );
        if (seawater_membership.medium > 0)
            output.high = fmax(
                    output.high,
                    fuzzyAnd(
                        receiver_membership.high,
                        seawater_membership.medium
                    )
            );

        if (seawater_membership.low > 0)
            output.medium = fmax(
                    output.medium,
                    fuzzyAnd(
                        receiver_membership.high,
                        seawater_membership.low
                    )
            );
        if (seawater_membership.very_low > 0)
            output.low = fmax(
                    output.low,
                    fuzzyAnd(
                        receiver_membership.high,
                        seawater_membership.very_low
                    )
            );
    }

    if(receiver_membership.very_high > 0) {
        if (seawater_membership.very_high > 0)
            output.extremely_high = fmax(
                    output.extremely_high,
                    fuzzyAnd(
                        receiver_membership.very_high,
                        seawater_membership.very_high
                    )
            );
        if (seawater_membership.high > 0)
            output.very_high = fmax(
                    output.very_high,
                    fuzzyAnd(
                        receiver_membership.very_high,
                        seawater_membership.high
                    )
            );
        if (seawater_membership.medium > 0)
            output.high = fmax(
                    output.high,
                    fuzzyAnd(
                        receiver_membership.very_high,
                        seawater_membership.medium
                    )
            );

        if (seawater_membership.low > 0)
            output.medium = fmax(
                    output.medium,
                    fuzzyAnd(
                        receiver_membership.very_high,
                        seawater_membership.low
                    )
            );
        if (seawater_membership.very_low > 0)
            output.low = fmax(
                    output.low,
                    fuzzyAnd(
                        receiver_membership.very_high,
                        seawater_membership.very_low
                    )
            );
    }
}

/**
 * Using mamdani centroid defuzzification
 * method finds the flow rate at which the
 * pump should dispense
 * @param seawater_temperature
 * @param receiver_temperature
 * @return crisp flow rate
 */
float FuzzyController::defuzzify(float seawater_temperature, float receiver_temperature) {
    fuzzy_output fo = setFlowMembership(seawater_temperature, receiver_temperature);
    if (!fo.active)
        return 0;

    float numerator = 0.0;
    float denominator = 0.0;

    for (float x = MIN_FLOW_RATE; x < MAMDANI_MAX_LIMIT; x+= 0.02){
        float s = getMax(6,
                         fmin(
                                 fo.very_low,
                                 getZShapeDistributionValue(
                                         x,
                                         output_limits.very_low_a,
                                         output_limits.very_low_b
                                         )
                                 ),
                         fmin(
                                 fo.low,
                                 getTriangularShapeDistributionValue(
                                         x,
                                         output_limits.low_a,
                                         output_limits.low_b
                                         )
                                 ),
                         fmin(
                                 fo.medium,
                                 getTriangularShapeDistributionValue(
                                         x,
                                         output_limits.medium_a,
                                         output_limits.medium_b
                                         )
                                 ),
                         fmin(
                                 fo.high,
                                 getTriangularShapeDistributionValue(
                                         x,
                                         output_limits.high_a,
                                         output_limits.high_b
                                         )
                                 ),
                         fmin(
                                 fo.very_high,
                                 getTriangularShapeDistributionValue(
                                         x,
                                         output_limits.very_high_a,
                                         output_limits.very_high_b
                                         )
                                 ),
                         fmin(
                                 fo.extremely_high,
                                 getZShapeDistributionValue(x,
                                        output_limits.extremely_high_a,
                                        output_limits.extremely_high_b
                                        )
                                 )
                         );
        numerator += x * s;
        denominator += s;
    }

    if (denominator > 0)
        return numerator / denominator;
    return 0.0;
}

/**
 * starts the fuzzy control using both parameters,
 * also uses \p heated_water_temperature to pump
 * at maximum speed if the temperature raises too high
 * @param seawater_temperature
 * @param receiver_temperature
 * @param heated_water_temperature
 */
void FuzzyController::handlePumpFlow(
        float seawater_temperature,
        float receiver_temperature,
        float heated_water_temperature) {
    if (heated_water_temperature >= MAX_TEMPERATURE_ALLOWED) {
        pmp.dispenseAtFlowRate(pmp.getMaxFlowRate() - 1.0);
        return;
    }
    flow_rate = defuzzify(seawater_temperature, receiver_temperature);
    if (flow_rate < MIN_FLOW_RATE || flow_rate > pmp.getMaxFlowRate()) {
        flow_rate = 0;
        pmp.stopDispensing();
        return;
    }
    pmp.dispenseAtFlowRate(flow_rate);
}

float FuzzyController::getFlowRate() {
    return flow_rate;
}