#include "HeatedWaterPump.h"

/**
 * Starts flow for about HW_PUMPING_TIME ms
 * if more than HW_FORCE_PUMPING_TIME ms have
 * passed or water temperature decreases below
 * HW_EVACUATION_TEMPERATURE celsius
 */
void HeatedWaterPump::startFlow() {

    if (is_pumping) {
        if (millis() - last_on > HW_PUMPING_TIME)
            deactivateRelay();
        return;
    }

    if (ezoPmp.getCurrentStatus()) {
        if (millis() - last_on > HW_FORCE_PUMPING_TIME) {
            activateRelay();
            return;
        }
    }

    if (ezoPmp.getVolumeDispensed() > 0) {
        if (temperatureSensor.getTemperature() < HW_EVACUATION_TEMPERATURE){
            activateRelay();
            ezoPmp.clearVolumeDispensed();
            return;
        }
    }

    deactivateRelay();
}