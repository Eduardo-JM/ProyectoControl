#include "DistilledWaterPump.h"

/**
 * Starts flow for about DW_PUMPING_TIME ms
 * if more than DW_FORCE_PUMPING_TIME ms have
 * passed or water level raises above
 * the sensor level activation height.
 */
void DistilledWaterPump::startFlow() {

    if (is_pumping) {
        if (millis() - last_on > DW_PUMPING_TIME)
            deactivateRelay();
        return;
    }

    if (levelSensor.readLevelSensor()) {
        activateRelay();
        return;
    }

    if (ezoPmp.getCurrentStatus()) {
        if (millis() - last_on > DW_FORCE_PUMPING_TIME) {
            activateRelay();
            return;
        }
    }

    deactivateRelay();
}