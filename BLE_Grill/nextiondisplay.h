#ifndef NEXTIONDISPLAY_H
#define NEXTIONDISPLAY_H

#include "hmi.h"

class NextionDisplay
{
public:
    NextionDisplay(HMISerial &uart, uint32_t baud = 9600);

    void updateData();
    void checkSerial();

private:
    HMI nextionHmi;

    void setSensorTemp(byte sensorId, char* data);
};

#endif // NEXTIONDISPLAY_H
