#include <Arduino.h> // main arduino library
#include <Stopwatch.h>
#include <User_Parameter.h>
#include <Ventilation_Modes.h>

User_Parameter allUserParams[(int) UP::LAST_PARAM_LABEL];
//Display(lcd, allUserParams, VC_CMV, );

const VentilationMode * pointer=&VC_CMV;

diagnosticParameters_t diagnosticParameters;

void setup(){


    allUserParams[(int) UP::RESPIRATORY_RATE] = User_Parameter(15, 5, 35, "f"); //  breaths per minute
    allUserParams[(int) UP::TIDAL_VOLUME] = User_Parameter(250, 0, 650, "VT"); // milliliters
    allUserParams[(int) UP::T_IN] = User_Parameter(2, 0.6, 4,"Ti"); // Inspiration time
    allUserParams[(int) UP::INSPIRATORY_PRESSURE] = User_Parameter(20, 5, 50, "Paw"); //  millibar
    allUserParams[(int) UP::FLOW] = User_Parameter(20, 5, 50,"Flo"); //  milliliters per second
    allUserParams[(int) UP::D_PRESSURE_SUPP] = User_Parameter(20, 5, 50, "dPs"); //  millibar
    allUserParams[(int) UP::PRESSURE_TRIGGER_THRESHOLD] = User_Parameter(5, 5, 50, "Ptr"); //  millibar per second
    allUserParams[(int) UP::FLOW_TRIGGER_THRESHOLD] = User_Parameter(20, 5, 50, "Ftr"); //  milliliters per second

    Serial.begin(115200);

}

void loop(){
    diagnosticParameters.flow.getValue()


}