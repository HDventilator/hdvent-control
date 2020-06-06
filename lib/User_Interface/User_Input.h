//
// Created by david on 04.06.20.
//

#ifndef HDVENT_CONTROL_USER_INPUT_H
#define HDVENT_CONTROL_USER_INPUT_H

#include <User_Parameter.h>
#include <Arduino.h>
#include <LiquidCrystal.h>
#include "Stopwatch.h"
#include <Ventilation_Modes.h>

const unsigned int ALLOWED_USER_EDIT_TIME = 15000; //milliseconds

class User_Input {
public:
    User_Input(User_Parameter *allUserParameters, VentilationMode *mode, bool *do_save);
    enum input_state_t {EDIT_SETTINGS, VIEW_SETTINGS, SAVE_SETTINGS, ENTER_EDIT, EDIT_VENTILATION_MODE, ENTER_VIEW_SETTINGS};
    void update();
private:
    input_state_t _inputState;
public:
    input_state_t getInputState() const;
    Stopwatch _stopwatch;

private:
    User_Parameter* _allUserParameters;
    VentilationMode* _mode;
    bool* _do_save;

};


#endif //HDVENT_CONTROL_USER_INPUT_H
