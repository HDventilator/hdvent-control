//
// Created by david on 08.01.21.
//

#ifndef HDVENT_CONTROL_WRITE_DATA_H
#define HDVENT_CONTROL_WRITE_DATA_H

#include <PacketSerial.h>
#include <Serial_Protocol.h>
#include <Diagnostic_Parameter.h>
#include <User_Parameter.h>
#include <CRC32.h>

template <int idLength, int prefixLength, int packageSize>
class PacketSerialID : public PacketSerial {
public:
    PacketSerialID(){}

    void write(const char* prefix, const char* identifier, float value){
        package_struct_float_t package = constructFloatPackage(prefix, identifier, value);
        // Use either this to get byte string
        uint8_t tmp[packageSize];
        memcpy(tmp, &package, packageSize);
        // or this:
        //const char* tmp = reinterpret_cast<char*>(&package);
        // Write data to serial port
        send(tmp, packageSize);
    }

    void write(Diagnostic_Parameter param){
        write(DIAGNOSTIC_PARAMETER_ID_PREFIX, param.getIdentifier(), param.getValue());
    }

    void write(User_Parameter param){
        write(USER_PARAMETER_VALUE_ID_PREFIX, param.lcdString, param.getValue());
        write(USER_PARAMETER_MIN_ID_PREFIX, param.lcdString, param.getMin());
        write(USER_PARAMETER_MAX_ID_PREFIX, param.lcdString, param.getMax());
    }

private:
    static package_struct_float_t constructFloatPackage(const char* prefix, const char* identifier, float value){
        package_struct_float_t dataPackage{};
        int index = 0;
        for (int i=0; i<prefixLength; i++){
            dataPackage.identifier[index]=identifier[index];
            index++;
        }
        for (int i=0; i<idLength-prefixLength; i++){
            dataPackage.identifier[index]=identifier[index];
            index++;
        }
        dataPackage.value = value;
        CRC32 crc;
        crc.update((uint8_t*) &dataPackage, IDENTIFIER_LENGTH+4);
        dataPackage.checksum = crc.finalize();
        return dataPackage;
    }
};

#endif //HDVENT_CONTROL_WRITE_DATA_H
