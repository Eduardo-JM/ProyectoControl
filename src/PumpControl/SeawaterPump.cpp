#include "SeawaterPump.h"

/**
 * Starts flow if the level sensor detects
 * a low water level
 */
void SeawaterPump::startFlow() {
    if (levelSensor.readLevelSensor())
        deactivateRelay();
    else
        activateRelay();
}