//
// Created by david on 17.04.20.
//

#ifndef HDVENT_CONTROL_VENTILATION_MODES_H
#define HDVENT_CONTROL_VENTILATION_MODES_H
#include "Trigger.h"
#include <Diagnostic_Parameter.h>



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
    ANGLE,
    LAST_PARAM_LABEL=11
};
/*
struct diagnosticParameters_t {
    Diagnostic_Parameter peep;
    Diagnostic_Parameter tidalVolume;
    Diagnostic_Parameter flow;
    Diagnostic_Parameter airwayPressure;
    Diagnostic_Parameter respiratoryRate;
    Diagnostic_Parameter plateauPressure;
    Diagnostic_Parameter meanPressure;
    Diagnostic_Parameter minuteVolume;
    Diagnostic_Parameter pressureChange; //millibar per second
};
*/
typedef bool (*trigger_func_t)();

struct Triggers{
    static bool inspirationAttempt();
    static bool pressureDrop();
    static bool flowIncrease();
    static bool respiratoryRate();
    static bool inspirationTime();
    static bool angleReached();
};

enum struct ControlMode { PC, VC, VN };

struct PID_parameters_t {
    float k_p;
    float k_i;
    float k_d;
};

const PID_parameters_t pidParams_PC{.k_p=1.0, .k_i=0.3, .k_d=0};
const PID_parameters_t pidParams_VC{.k_p=2.0, .k_i=0.3, .k_d=0};
const PID_parameters_t pidParams_VN{.k_p=1.0, .k_i=0., .k_d=0};

bool alwaysFalseTrigger();

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
    VentilationMode(ControlMode control, UP userSetParametersSelection[], int nUserSetParameters,
                    trigger_func_t inspirationTriggersSelection[], int nInspirationTriggers,
                    trigger_func_t expirationTriggersSelection[], int nExpirationTriggers,
                    char *identifier);

    UP parameters[(int)UP::LAST_PARAM_LABEL];
    trigger_func_t expirationTriggers[NUMBER_TRIGGERS];
    trigger_func_t inspirationTriggers[NUMBER_TRIGGERS];
    ControlMode controlMode;
    PID_parameters_t pidParameters;
    uint8_t nParams;
    package_struct_4char_t _package;

};

const VentilationMode VC_CMV = VentilationMode(
        ControlMode::VC,
        (UP[]) {
                UP::RESPIRATORY_RATE,
                UP::TIDAL_VOLUME,
                UP::T_IN,
                UP::FLOW}, 4,
        (trigger_func_t[]) {Triggers::respiratoryRate}, 1,
        (trigger_func_t[]) {Triggers::inspirationTime}, 1, nullptr);

const VentilationMode OL_CMV = VentilationMode(
        ControlMode::VN,
        (UP[]) {
                UP::RESPIRATORY_RATE,
                UP::TIDAL_VOLUME,
                UP::T_IN}, 3,
        (trigger_func_t[]) {Triggers::respiratoryRate}, 1,
        (trigger_func_t[]) {Triggers::inspirationTime}, 1, nullptr);

const VentilationMode PC_CMV = VentilationMode(
        ControlMode::VN,
        (UP[]) {
                UP::INSPIRATORY_PRESSURE,
                UP::RESPIRATORY_RATE,
                UP::TIDAL_VOLUME,
                UP::T_IN}, 3,
        (trigger_func_t[]) {Triggers::respiratoryRate}, 1,
        (trigger_func_t[]) {Triggers::inspirationTime}, 1, nullptr);


#endif //HDVENT_CONTROL_VENTILATION_MODES_H