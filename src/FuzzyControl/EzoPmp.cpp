#include "EzoPmp.h"

EzoPmp::EzoPmp() {
    max_flow_rate = MAX_FLOW_RATE;
    initialized = false;
    setMaxFlowRate();
    current_status = false;
    clearVolumeDispensed();
    executeCommand("C,*");
}

/**
 * This method sends a string to the EZO PMP
 * this method doesn't handle any errors.
 */
void EzoPmp::executeCommand(String command) {
    Serial3.print(command);
    Serial3.print(CARRIAGE_RETURN);
}

/**
 * Sets PumpResponse.success to true or
 * false depending success status
 * also sets numeric response if theres one
 * @param str EZO-PMP response
 */
void EzoPmp::setResponse(String str){
    response.numeric = NAN;

    if (str.indexOf("OK") != -1)
        response.success = true;
    else if (str.indexOf("ER") != -1)
        response.success = false;

    if (str.indexOf(",") != -1){
        String valueStr = str.substring(str.indexOf(',') + 1);
        if (isdigit(valueStr[0]) || valueStr[0] == '-') {
            response.numeric = valueStr.toFloat();
            if (!initialized) {
                max_flow_rate = response.numeric;
                initialized = true;
            }
        }
    }
}

/**
 * sets the maximum flow rate of the pump
 */
void EzoPmp::setMaxFlowRate() {
    if (initialized)
        return;
    executeCommand("DC,?");
}

/**
 * Abruptly stops dispensing water
 */
void EzoPmp::stopDispensing() {
    executeCommand("X");
    current_status = false;
}

/**
 * Starts dispensing water at the specified flow
 * rate indefinitely
 * @param flow_rate
 */
void EzoPmp::dispenseAtFlowRate(float flow_rate) {
    if (flow_rate < MIN_FLOW_RATE) {
        stopDispensing();
        return;
    }
    String str = "DC,"+ String(flow_rate) + ",*";
    executeCommand(str);
    current_status = true;
}

/**
 * @return max flow rate the pump can dispense
 */
float EzoPmp::getMaxFlowRate() {
    return max_flow_rate;
}

/**
 * @return EZO-PMP volume dispensed since
 * last caché clear
 */
float EzoPmp::getVolumeDispensed() {
    return response.numeric;
}

/**
 * Clears EZO-PMP dispensed volume
 */
void EzoPmp::clearVolumeDispensed() {
    executeCommand("Clear");
}

/**
 * @return true if pump is dispensing water,
 * false otherwise
 */
bool EzoPmp::getCurrentStatus() {
    return current_status;
}