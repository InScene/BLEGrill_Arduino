#include "nextiondisplay.h"
#include "BLEGrill.h"

#define TEMP1_TEXT_FIELD_ID "temp1"
#define TEMP2_TEXT_FIELD_ID "temp2"
#define TEMP3_TEXT_FIELD_ID "temp3"
#define TEMP4_TEXT_FIELD_ID "temp4"

NextionDisplay::NextionDisplay(HMISerial &uart, uint32_t baud) :
    nextionHmi(uart, baud)
{

}

void NextionDisplay::updateData()
{
    for(int i=0; i< MAX_TEMP_SENSORS; i++)
    {
        TempSensor* sensor = BLEGrill::instance()->getSensorByNb(0);
        char temp[8] = {0};
        sensor->getTemperatureAsChar(temp);
        this->setSensorTemp(i, temp );
    }
}

void NextionDisplay::checkSerial()
{

}

void NextionDisplay::setSensorTemp(byte sensorId, char *data)
{
    char textId[6] = {0};

    switch(sensorId)
    {
        case 0:
            strcat(textId, TEMP1_TEXT_FIELD_ID);
            break;
        case 1:
            strcat(textId, TEMP2_TEXT_FIELD_ID);
            break;
        case 2:
            strcat(textId, TEMP3_TEXT_FIELD_ID);
            break;
        case 3:
            strcat(textId, TEMP4_TEXT_FIELD_ID);
            break;
    }

    nextionHmi.textEdit(textId, data );

}

