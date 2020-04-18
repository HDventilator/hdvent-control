//
// Created by david on 17.04.20.
//

#ifndef HDVENT_CONTROL_VENTILATION_MODES_H
#define HDVENT_CONTROL_VENTILATION_MODES_H


enum struct diagnosticParameters_t {
    TIDAL_VOLUME,
    FLOW_RATE,
    INSPIRATORY_PRESSURE,
    RESPIRATORY_RATE,
    PLATEAU_PRESSURE,
    MINUTE_VOLUME,
    PEEP,
    LAST_PARAM_LABEL = 8
};

enum struct userSetParameters_t {
    BLANK,
    INSPIRATORY_PRESSURE,
    TIDAL_VOLUME,
    RESPIRATORY_RATE,
    T_IN,
    FLOW,
    SLOPE,
    D_PRESSURE_SUPP,
    LAST_PARAM_LABEL=9
};

enum struct triggerType_t{
    BLANK,
    INSPIRATION_ATTEMPT,
    TIME_AFTER_PREVIOUS_INSPIRATION,
    TIME_AFTER_CURRENT_INSPIRATION,
    LAST_PARAM_LABEL=2,
};


struct VentilationMode {
    userSetParameters_t userSetParameters[(int)userSetParameters_t::LAST_PARAM_LABEL];
    triggerType_t inspirationTriggers[(int)triggerType_t::LAST_PARAM_LABEL];
    triggerType_t expirationTriggers[(int)triggerType_t::LAST_PARAM_LABEL];
};

const VentilationMode VC_CMV{
        {userSetParameters_t::RESPIRATORY_RATE,
                userSetParameters_t::TIDAL_VOLUME,
                userSetParameters_t::T_IN,
                userSetParameters_t::FLOW
        },
        {triggerType_t::TIME_AFTER_PREVIOUS_INSPIRATION},
        {triggerType_t::TIME_AFTER_CURRENT_INSPIRATION},
};


#endif //HDVENT_CONTROL_VENTILATION_MODES_H
