#include "TempSensor.h"
#include <math.h>
#include "constant.h"
#include <inttypes.h>
#include <Arduino.h>
#include <avr/pgmspace.h>

TempSensor::TempSensor(const uint8_t pin, const uint8_t sensorNb, const SENSORTYPE type,
                       uint8_t pipeTempSetId, uint8_t pipeTxId,
                       uint8_t pipeAlarmSettingSetId, uint8_t pipeSensorConfigSetId) :
    _temperature(TEMPERATURE_UNDEFINED),
    _pin(pin),
    _sensorNb(sensorNb),
    _type(type),
    _state(SENSOR_OFF),
    _alarmType(NO_ALARM),
    _alarmState(STATE_NO_ALARM),
    _enabled(false),
    _highTempBorder(0xFFFF),
    _lowTempBorder(0),
    _pipeSetId(pipeTempSetId),
    _pipeTxId(pipeTxId),
    _pipeAlarmSettingsSetId(pipeAlarmSettingSetId),
    _pipeSensorConfigSetId(pipeSensorConfigSetId),
    _alarmIsActive(false)
{
}

void TempSensor::activateSensor(const bool activate)
{
    if(_enabled != activate)
    {
        if(activate)
        {
            _state = SENSOR_OK;
            _enabled = true;

#ifdef DEBUG
            Serial.print("Sensor enabled: ");
            Serial.println(_sensorNb,DEC);
#endif
        }
        else
        {
            _state = SENSOR_OFF;
            _enabled = false;
            _alarmIsActive = false;
            _alarmState = STATE_NO_ALARM;

#ifdef DEBUG
            Serial.print("Sensor disabled: ");
            Serial.println(_sensorNb,DEC);
#endif
        }
    }
}

void TempSensor::measureTemperature()
{
    if(_enabled)
    {
        doMeasurement();
        checkAlarm();
    }
}

void TempSensor::setSensorType(uint8_t type)
{
    switch(type)
    {
        case ACURITE:
            _type = ACURITE;
            break;
        case FANTAST:
            _type = FANTAST;
            break;
        case ROSENSTEIN:
            _type = ROSENSTEIN;
            break;
        case MAVERICK:
            _type = MAVERICK;
            break;
        default:
            _type = MAVERICK;
            break;
    }
}

void TempSensor::setHighTempBorder(const uint16_t tempValue)
{
    /* Set value only if not smaller than low temperature + hysteresis */
    if(tempValue > (_lowTempBorder + _TEMP_ALARM_HYSTERESIS))
    {
        _highTempBorder = tempValue;
        #ifdef DEBUG
        Serial.print("High temp set. Temp: ");
        Serial.print(tempValue,DEC);
        Serial.print(", sensor: ");
        Serial.println(_sensorNb,DEC);
        #endif
    }
    else
    {
        Serial.print("High temp not set, to low:");
        Serial.print(tempValue,DEC);
        Serial.print(", sensor: ");
        Serial.println(_sensorNb,DEC);
    }
}

void TempSensor::setLowTempBorder(const uint16_t tempValue)
{
    /* Set value only if not smaller than low temperature + hysteresis */
    if(tempValue < (_highTempBorder + _TEMP_ALARM_HYSTERESIS))
    {
        _lowTempBorder = tempValue;
        #ifdef DEBUG
        Serial.print("Low temp set. Temp: ");
        Serial.print(tempValue,DEC);
        Serial.print(", sensor: ");
        Serial.println(_sensorNb,DEC);
        #endif
    }
    else
    {
        Serial.print("Low temp not set, to high:");
        Serial.print(tempValue,DEC);
        Serial.print(", sensor: ");
        Serial.println(_sensorNb,DEC);
    }
}

void TempSensor::setAlarmType(const uint8_t alarmType)
{
    switch(alarmType)
    {
        case 0:
            _alarmType = NO_ALARM;
            break;
        case 1:
            _alarmType = HIGH_ALARM;
            break;
        case 2:
            _alarmType = LOW_ALARM;
            break;
        case 3:
            _alarmType = HIGH_LOW_ALARM;
            break;
        default:
            _alarmType = HIGH_LOW_ALARM;
            break;
    }

    #ifdef DEBUG
    Serial.print("Alarm type set: ");
    Serial.print(_alarmType,DEC);
    Serial.print(", sensor: ");
    Serial.println(_sensorNb,DEC);
    #endif
}

void TempSensor::getTempMeasurementBuffer(uint8_t *dest, uint8_t *size) const
{
    *size = 4;

    dest[0] = 0; /* Status Bits, auf °C gesetzt */
    dest[1] = _temperature;   /* Low Byte Temperature */
    dest[2] = (_temperature >> 8); /* High Byte Temperature */
    dest[3] = 0;
}

uint8_t TempSensor::getSensorNb() const
{
    return _sensorNb;
}

uint16_t TempSensor::getTemperature() const
{
    return _temperature;
}

uint16_t TempSensor::getHighTempBorder() const
{
    return _highTempBorder;
}

uint16_t TempSensor::getLowTempBorder() const
{
    return _lowTempBorder;
}

uint8_t TempSensor::getPipeTempSetId() const
{
    return _pipeSetId;
}

uint8_t TempSensor::getPipeTxId() const
{
    return _pipeTxId;
}

uint8_t TempSensor::getPipeAlarmSettingsSetId() const
{
    return _pipeAlarmSettingsSetId;
}

uint8_t TempSensor::getPipeSensorConfigSetId() const
{
    return _pipeSensorConfigSetId;
}

TempSensor::SENSORSTATE TempSensor::getState() const
{
    return _state;
}

bool TempSensor::hasAlarm() const
{
    bool alarm = false;

    if( _alarmState != STATE_NO_ALARM)
    {
        alarm = true;
    }
    else
    {
        alarm = false;
    }

    return alarm;
}

bool TempSensor::sensorIsActive() const
{
    return _enabled;
}

TempSensor::SENSOR_ALARM_TYPE TempSensor::getAlarmType() const
{
    return _alarmType;
}

TempSensor::SENSORTYPE TempSensor::getSensorType() const
{
    return _type;
}

TempSensor::SENSOR_ALARM_STATE TempSensor::getAlarmState() const
{
    return _alarmState;
}

void TempSensor::doMeasurement()
{
    const uint8_t nbMeasures = 10;
    const uint8_t measureResistor = 200; /* 200kOhm */
    uint16_t measurements = 0;
    double value = 0;
    uint16_t temperature = 0;
    double Rt = 0;

    /* Do a number of readings to reduce measurement errors */
    for(int iterations = 0; iterations<nbMeasures; iterations++)
    {
        /* One Reading takes about 0.0001 s */
        measurements += analogRead(_pin);
    }

    if( measurements != 0)
    {
        /* Calculate middle value about all measurements */
        value = round(measurements / nbMeasures);
        Rt = measureResistor *((1024.0/value) - 1);
        temperature = convertValueIntoTemperature(Rt);

        /* Check if the measurement is correct */
        if (temperature != 999.9)
        {
            /* Measurement is correct */
            _temperature = temperature;
            _state = SENSOR_OK;
        }
        else
        {
            /* A error is occured */
            _temperature = TEMPERATURE_UNDEFINED;
            _state = MEASUREMENT_ERR;
            Serial.print("Temp measurement error on sensor: ");
            Serial.println(_sensorNb,DEC);
        }
    }
    else
    {
        _temperature = TEMPERATURE_UNDEFINED;
        _state = NO_SENSOR;
        Serial.print("No Tempsensor on sensor: ");
        Serial.println(_sensorNb,DEC);
    }

    #ifdef DEBUG
    Serial.print(F("AD Data: "));
    Serial.print(value,DEC);
    Serial.print(F(", Rt Data: "));
    Serial.print(Rt,DEC);
    Serial.print(F(", Temp Data: "));
    Serial.print(_temperature,DEC);
    Serial.print(", sensor: ");
    Serial.println(_sensorNb,DEC);
    #endif
}

uint16_t TempSensor::convertValueIntoTemperature(const double Rt) const
{
    double a;
    double b;
    double c;
    uint16_t Rn;

    switch(_type)
    {
        case ACURITE:
            /* Kennlinienkonstanten Sensor ACURITE/Ventus */
            a = 0.003344;
            b = 0.000251911;
            c = 0.00000351094;
            Rn = 47;
            break;
        case FANTAST:
            /* Kennlinienkonstanten Sensor IKEA FANTAST */
            a = 0.003339;
            b = 0.000251911;
            c = 0.00000351094;
            Rn = 47;
            break;
        case ROSENSTEIN:
            /* Kennlinienkonstanten Rosenstein und Soehne RF-T0601 */
            a = 0.003362;
            b = 0.000251911;
            c = 0.00000351094;
            Rn = 100;
            break;
        case MAVERICK:
            /* Kennlinienkonstanten Sensor MAVERICK */
            a = 0.0033354016;
            b = 0.000225;
            c = 0.00000251094;
            Rn = 925;
            break;
    }

    double v = log(Rt/Rn);
    double temp = (1/(a + b*v + c*v*v)) - 273;

    /* Wandle Zahl in Festkommazahl mit einer Nachkommastelle */
    uint16_t temperature = temp * 100;

    return temperature;
}

bool TempSensor::isHighAlarm() const
{
    bool alarm = false;

    /* If alarm not active, do check only against border */
    if(!_alarmIsActive)
    {
        /* Set alarm only if alarm type is selected */
        if( ((_alarmType == HIGH_ALARM) ||
             (_alarmType == HIGH_LOW_ALARM) ) &&
            (getTemperature() > _highTempBorder) )
        {
            alarm = true;
        }
    }
    else
    {
        /* Alarm is active, so we must use hysteresis */
        /* Alarm only still active when state correct and temperature higher than border + hysteresis */
        if( ((_alarmType == HIGH_ALARM) ||
             (_alarmType == HIGH_LOW_ALARM) ) &&
            (getTemperature() > (_highTempBorder-_TEMP_ALARM_HYSTERESIS)) )
        {
            alarm = true;
        }
    }

    return alarm;
}

bool TempSensor::isLowAlarm() const
{
    bool alarm = false;

    /* If alarm not active, do check only against border */
    if( !_alarmIsActive )
    {
        /* Set alarm only if alarm type is selected */
        if( ((_alarmType == LOW_ALARM) ||
             (_alarmType == HIGH_LOW_ALARM) ) &&
            (getTemperature() < _lowTempBorder) )
        {
            alarm = true;
        }
    }
    else
    {
        /* Alarm is active, so we must use hysteresis */
        /* Alarm only still active when state correct and temperature lower than border + hysteresis */
        if( ((_alarmType == LOW_ALARM) ||
             (_alarmType == HIGH_LOW_ALARM) ) &&
            (getTemperature() > (_lowTempBorder+_TEMP_ALARM_HYSTERESIS)) )
        {
            alarm = true;
        }
    }

    return alarm;
}

void TempSensor::checkAlarm()
{
    /* Alarm only possible when sensor active */
    if(_enabled)
    {
        /* Set alarm only if alarm type is selected */
        if( isHighAlarm() )
        {
            _alarmState = STATE_HIGH_ALARM;
        }
        else if( isLowAlarm() )
        {
            _alarmState = STATE_LOW_ALARM;
        }
        else if( _state == NO_SENSOR)
        {
            _alarmState = STATE_NO_SENSOR_ALARM;
        }
        else if(_state == MEASUREMENT_ERR)
        {
            _alarmState = STATE_MEASURE_ERR_ALARM;
        }
        else
        {
            _alarmState = STATE_NO_ALARM;
        }
    }
}

