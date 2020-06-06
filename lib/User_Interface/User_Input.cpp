//
// Created by david on 04.06.20.
//

#include "User_Input.h"
#include "User_Parameter.h"
#include <Ventilation_Modes.h>
#include <Arduino.h>
#include <LiquidCrystal.h>

User_Input::User_Input(User_Parameter *allUserParameters, VentilationMode *mode, bool *do_save) {
    _allUserParameters = allUserParameters;
    _mode = mode;
    _do_save = do_save;
    _inputState = VIEW_SETTINGS;

}

void User_Input::update() {
    switch(_inputState){
        case EDIT_VENTILATION_MODE:
            break;

        case VIEW_SETTINGS:
            {
                bool valueChanged=false;
                for ( int i=0; !valueChanged &&  i<(_mode->nParams); i++)
                {
                    valueChanged = valueChanged || _allUserParameters[(int)_mode->parameters[i]].hasChanged();
                }
                if (valueChanged) {
                    _inputState = ENTER_EDIT;
                }
                break;
            }

        case ENTER_EDIT:
            _inputState = EDIT_SETTINGS;
            _stopwatch.start();
            break;

        case EDIT_SETTINGS:
            if (*_do_save){
                _inputState = SAVE_SETTINGS;
            }
            else if (_stopwatch.getElapsedTime() > ALLOWED_USER_EDIT_TIME){
                _inputState = VIEW_SETTINGS;
            }
            else{
                _inputState = EDIT_SETTINGS;
            }
            break;

        case SAVE_SETTINGS:
            for ( int i=0;  i<(_mode->nParams); i++)
            {
                _allUserParameters[(int)_mode->parameters[i]].saveValue();
            }
            _inputState = VIEW_SETTINGS;
            break;

    }
}

User_Input::input_state_t User_Input::getInputState() const {
    return _inputState;
}
