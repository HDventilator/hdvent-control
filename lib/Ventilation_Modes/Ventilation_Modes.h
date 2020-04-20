//
// Created by david on 17.04.20.
//

#ifndef HDVENT_CONTROL_VENTILATION_MODES_H
#define HDVENT_CONTROL_VENTILATION_MODES_H
#include "Trigger.h"
enum struct UP {
    BLANK,
    INSPIRATORY_PRESSURE,
    TIDAL_VOLUME,
    RESPIRATORY_RATE,
    T_IN,
    FLOW,
    SLOPE,
    FLOW_TRIGGER_THRESHOLD,
    PRESSURE_TRIGGER_THRESHOLD,
    D_PRESSURE_SUPP,
    LAST_PARAM_LABEL=10
};
typedef bool (*trigger_func_t)();

struct Triggers{
    static bool inspirationAttempt();
    static bool pressureDrop();
    static bool flowIncrease();
    static bool respiratoryRate();
    static bool inspirationTime();
};


bool alwaysFalseTrigger(){
    return false;
}

template <typename T>
void fillArray(T A[], int N, T a[], int n, T fillValue) {
    for (int i = 0; i < N; i++) {
        if (i < n) {
            A[i] = a[i];
        } else {
            A[i] = fillValue;
        }
    }
}

const uint8_t NUMBER_TRIGGERS=5;
struct VentilationMode {
    VentilationMode(
            UP userSetParametersSelection[], int nUserSetParameters,
            trigger_func_t inspirationTriggersSelection[], int nInspirationTriggers,
            trigger_func_t expirationTriggersSelection[], int nExpirationTriggers){
        fillArray(parameters, (int)UP::LAST_PARAM_LABEL, userSetParametersSelection, nUserSetParameters, UP::BLANK);
        fillArray(inspirationTriggers, NUMBER_TRIGGERS, inspirationTriggersSelection, nInspirationTriggers, &alwaysFalseTrigger);
        fillArray(expirationTriggers, NUMBER_TRIGGERS, expirationTriggersSelection, nExpirationTriggers, &alwaysFalseTrigger);
    }

    UP parameters[(int)UP::LAST_PARAM_LABEL];
    trigger_func_t expirationTriggers[NUMBER_TRIGGERS];
    trigger_func_t inspirationTriggers[NUMBER_TRIGGERS];
    bool expirationTrigger(){return anyTrue(expirationTriggers);}
    bool inspirationTrigger(){return anyTrue(inspirationTriggers);}

};


const VentilationMode VC_CMV = VentilationMode(
        (UP[]) {
                UP::RESPIRATORY_RATE,
                UP::TIDAL_VOLUME,
                UP::T_IN,
                UP::FLOW}, 4,
        (trigger_func_t[]) {Triggers::respiratoryRate}, 1,
        (trigger_func_t[]) {Triggers::inspirationTime}, 1);

#endif //HDVENT_CONTROL_VENTILATION_MODES_H
