//
// Created by david on 26.04.20.
//

#include <Ventilation_Modes.h>

bool alwaysFalseTrigger(){
return false;
}

VentilationMode::VentilationMode(ControlMode control, UP userSetParametersSelection[], int nUserSetParameters,
                                 trigger_func_t inspirationTriggersSelection[], int nInspirationTriggers,
                                 trigger_func_t expirationTriggersSelection[], int nExpirationTriggers,
                                 char *identifier)
{
    //fillArray(parameters, (int)UP::LAST, userSetParametersSelection, nUserSetParameters, UP::BLANK);
    parameters = userSetParametersSelection;
    fillArray(inspirationTriggers, NUMBER_TRIGGERS, inspirationTriggersSelection, nInspirationTriggers, &alwaysFalseTrigger);
    fillArray(expirationTriggers, NUMBER_TRIGGERS, expirationTriggersSelection, nExpirationTriggers, &alwaysFalseTrigger);
    controlMode = control;
    nParams = nUserSetParameters;
    switch (control){
        case ControlMode::PC :
            pidParameters = pidParams_PC;
            break;
        case ControlMode::VC :
            pidParameters = pidParams_VC;
            break;
        case ControlMode::OL :
            pidParameters = pidParams_VN;
    }
    lcdString = identifier;

}

