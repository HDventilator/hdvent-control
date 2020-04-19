//
// Created by david on 17.04.20.
//

#ifndef HDVENT_CONTROL_VENTILATION_MODES_H
#define HDVENT_CONTROL_VENTILATION_MODES_H

enum struct userSetParameters_t {
    INSPIRATORY_PRESSURE,
    TIDAL_VOLUME,
    RESPIRATORY_RATE,
    T_IN,
    FLOW,
    SLOPE,
    D_PRESSURE_SUPP,
    LAST_PARAM_LABEL=9
};
typedef bool (*trigger_func_t)();

struct{
trigger_func_t pressureDrop;
trigger_func_t flowIncrease;
trigger_func_t respiratoryRate;
trigger_func_t inspirationTime;
} trigger;

enum struct triggerLabel{
        PRESSURE_DROP, FLOW_INCREASE, RESPIRATORY_RATE, INSPIRATION_TIME, LAST_LABEL};

trigger_func_t triggerFunctions[]={
        trigger.pressureDrop,  trigger.flowIncrease,  trigger.respiratoryRate, trigger.inspirationTime,
};

struct VentilationMode {
    userSetParameters_t userSetParameters[(int)userSetParameters_t::LAST_PARAM_LABEL];
    triggerLabel inspirationTriggers[(int)triggerLabel::LAST_LABEL];
    triggerLabel expirationTriggers[(int)triggerLabel::LAST_LABEL];
};


const VentilationMode VC_CMV{
        {userSetParameters_t::RESPIRATORY_RATE,
                userSetParameters_t::TIDAL_VOLUME,
                userSetParameters_t::T_IN,
                userSetParameters_t::FLOW
        },
        {triggerLabel::RESPIRATORY_RATE},
        {triggerLabel::INSPIRATION_TIME},
};

#endif //HDVENT_CONTROL_VENTILATION_MODES_H
