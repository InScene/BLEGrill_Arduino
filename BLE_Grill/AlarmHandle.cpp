#include "AlarmHandle.h"
#include "DeviceSettings.h"
#include "constant.h"
#include "BLEGrill.h"


AlarmHandle::AlarmHandle():
    _newAlarm(false)
{
    for(int i=0; i<MAX_TEMP_SENSORS; i++)
    {
        _sensorsAlarmState[i] = TempSensor::STATE_NO_ALARM;
    }
}

bool AlarmHandle::checkTempSensorAlarms()
{
    bool alarmFound = false;

    /* Check all sensor for alarms */
    for(int i=0; i<MAX_TEMP_SENSORS; i++)
    {
        TempSensor* sensor = BLEGrill::instance()->getSensorByNb(i);
        if( sensor )
        {
            TempSensor::SENSOR_ALARM_STATE alarm = sensor->getAlarmState();
            /* If no alarm, reset internal state for this sensor */
            if( alarm  == TempSensor::STATE_NO_ALARM)
            {
                _sensorsAlarmState[i] = TempSensor::STATE_NO_ALARM;
                #ifdef DEBUG
                Serial.print("No alarm for sensor: ");
                Serial.println(sensor->getSensorNb(),DEC);
                #endif
            }
            /* If alarm, check if it a new one */
            else if( alarm != _sensorsAlarmState[i] )
            {
                Serial.print("Sensor has new alarm. Sensor: ");
                Serial.print(sensor->getSensorNb(),DEC);
                Serial.print(", alarm: ");
                Serial.println(sensor->getAlarmState());

                alarmFound = true;
                _sensorsAlarmState[i] = sensor->getAlarmState();
                _newAlarm = true;

            }
            else
            {
                #ifdef DEBUG
                Serial.print("Old Alarm for sensor: ");
                Serial.println(sensor->getSensorNb(),DEC);
                #endif
                alarmFound = true;
                _newAlarm = false;
            }
        }
    }

    return alarmFound;
}

bool AlarmHandle::isNewAlarm()
{
    return _newAlarm;
}
