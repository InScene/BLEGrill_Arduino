#ifndef ALARMHANDLE_H
#define ALARMHANDLE_H

#include "TempSensor.h"
#include "constant.h"

class AlarmHandle
{
public:
    AlarmHandle();

    bool checkTempSensorAlarms();
    bool isNewAlarm();

private:
    TempSensor::SENSOR_ALARM_STATE _sensorsAlarmState[MAX_TEMP_SENSORS];
    bool _newAlarm;
};

#endif // ALARMHANDLE_H
