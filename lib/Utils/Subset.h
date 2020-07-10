//
// Created by david on 10.07.20.
//

#ifndef HDVENT_CONTROL_SUBSET_H
#define HDVENT_CONTROL_SUBSET_H


template <class value_t, class enum_t>
class Subset {
public:
    Subset(value_t* valuesArray, enum_t* indicesArray) {
        _valuesArray = valuesArray;
        _indicesArray = indicesArray;
    }


    value_t& operator[](int idx)  { return _valuesArray[(int)_indicesArray[idx]]; }
    const value_t& operator[](int idx) const { return _valuesArray[(int)_indicesArray[idx]]; }
//private:
    value_t * _valuesArray;
    enum_t * _indicesArray;
};


#endif //HDVENT_CONTROL_SUBSET_H
