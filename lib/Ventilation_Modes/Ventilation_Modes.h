//
// Created by david on 17.04.20.
//

#ifndef HDVENT_CONTROL_VENTILATION_MODES_H
#define HDVENT_CONTROL_VENTILATION_MODES_H
#include "Trigger.h"
#include <Diagnostic_Parameter.h>



enum struct UP {
    FIRST,
    INSPIRATORY_PRESSURE=FIRST,
    TIDAL_VOLUME,
    COMPRESSED_VOLUME_RATIO,
    RESPIRATORY_RATE,
    T_IN,
    FLOW,
    SLOPE_P,
    FLOW_TRIGGER_THRESHOLD,
    PRESSURE_TRIGGER_THRESHOLD,
    D_PRESSURE_SUPP,
    ANGLE,
    KP,
    KI,
    LAST=KI
};

typedef bool (*trigger_func_t)();

struct Triggers{
    static bool inspirationAttempt();
    static bool pressureDrop();
    static bool flowIncrease();
    static bool respiratoryRate();
    static bool inspirationTime();
    static bool angleReached();
    static bool tidalVolume();
};

enum struct ControlMode { PC=0, VC=1, OL=2 };
enum struct VentiModes {PC_CMV, OL_CMV, VC_CMV, LAST_LABEL};

struct PID_parameters_t {
    float k_p;
    float k_i;
    float k_d;
};

const PID_parameters_t pidParams_PC{.k_p=15, .k_i=5, .k_d=0};
const PID_parameters_t pidParams_VC{.k_p=15, .k_i=10, .k_d=0};
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

class VentilationMode {
public:
    VentilationMode(ControlMode control, UP userSetParametersSelection[], int nUserSetParameters,
                    trigger_func_t inspirationTriggersSelection[], int nInspirationTriggers,
                    trigger_func_t expirationTriggersSelection[], int nExpirationTriggers,
                    char *identifier);
    VentilationMode(){};

    UP* parameters;
    trigger_func_t expirationTriggers[NUMBER_TRIGGERS];
    trigger_func_t inspirationTriggers[NUMBER_TRIGGERS];
    ControlMode controlMode;
    PID_parameters_t pidParameters;
    uint8_t nParams;
    package_struct_4char_t _package;
    char * lcdString;


};

template<uint8_t N, typename VM_t>
class VentiModeContainer {
public:
    VentiModeContainer(VentilationMode *params) : nActive(N), params(params) {}
    VentiModeContainer() {
    }
    VentilationMode & operator[](VM_t idx)       { return params[static_cast<int>(idx)]; }
    const VentilationMode& operator[](VM_t idx)      const  { return params[static_cast<int>(idx)]; }


    int nActive{};
    VentilationMode& getActiveMode(){
        return params[_activeIndex];
    }
    VentilationMode& getSelectedMode(){
        return params[_selectedIndex];
    }
    int getActiveIndex(){
        return _activeIndex;
    }
    int getSelectedIndex(){
        return _selectedIndex;
    }
    void setActive(VM_t i){
        _activeIndex = static_cast<int>(i);
    }
    void setSelected(VM_t i){
        _selectedIndex = static_cast<int>(i);
    }
    void setActive(int i){
        _activeIndex = i;
    }
    void setSelected(int i){
        _selectedIndex = i;
    }
    void save(){
        _activeIndex=_selectedIndex;
    }
    
    VentilationMode params[N];
private:
    int _activeIndex;
    int _selectedIndex;


};

const VentilationMode VC_CMV(
        ControlMode::VC,
        (UP[]){
                UP::RESPIRATORY_RATE,
                UP::TIDAL_VOLUME,
                UP::T_IN,
                UP::FLOW,
                UP::KP,
                UP::KI
                }, 6,
        (trigger_func_t[]) {Triggers::respiratoryRate}, 1,
        (trigger_func_t[]) {Triggers::inspirationTime}, 1, "VC-CMV");

const VentilationMode OL_CMV (
        ControlMode::OL,
        (UP[]) {
                UP::RESPIRATORY_RATE,
                UP::COMPRESSED_VOLUME_RATIO,
                UP::T_IN}, 3,
        (trigger_func_t[]) {Triggers::respiratoryRate}, 1,
        (trigger_func_t[]) {Triggers::inspirationTime}, 1, "OL-CMV");

const VentilationMode PC_CMV (
        ControlMode::PC,
        (UP[]) {
                UP::INSPIRATORY_PRESSURE,
                UP::RESPIRATORY_RATE,
                UP::SLOPE_P,
                UP::T_IN}, 4,
        (trigger_func_t[]) {Triggers::respiratoryRate}, 1,
        (trigger_func_t[]) {Triggers::inspirationTime}, 1, "PC-CMV");





#endif //HDVENT_CONTROL_VENTILATION_MODES_H