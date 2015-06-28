#include "DeviceSettings.h"
#include "constant.h"
#include <Arduino.h>
#include <avr/pgmspace.h>

DeviceSettings* DeviceSettings::_instance = 0;

DeviceSettings *DeviceSettings::instance()
{
    if(_instance == 0)
    {
        _instance = new DeviceSettings();
    }

    return _instance;
}

DeviceSettings::DeviceSettings():
    _buzzerEnabled(true),
    _buzzerState(false),
    _alarmLedState(false),
    _statusLedState(false),
    _sensorPwrState(false)
{
}

void DeviceSettings::init(void)
{
    /* Set pin modes */
    pinMode(DIGITAL_OUT_SENSORS_VCC_PIN, OUTPUT);
    pinMode(DIGITAL_OUT_LED_ALARM_PIN, OUTPUT);
    pinMode(DIGITAL_OUT_BUZZER_ALARM_PIN, OUTPUT);
    pinMode(DIGITAL_OUT_LED_PWR_ON_PIN, OUTPUT);
    pinMode(DIGITAL_IN_SWITCH_PIN, INPUT);

    /* Set pin states */
    setStatusLedState(true);
    setAlarmLedState(false);
    setBuzzerState(false);
    setSensorPowerState(true);

    digitalWrite(DIGITAL_IN_SWITCH_PIN, HIGH);

    /* Set analog reference to External */
    analogReference(DEFAULT);
}

bool DeviceSettings::getBuzzerEnabled() const
{
    return _buzzerEnabled;
}

void DeviceSettings::setBuzzerEnabled(const bool buzzerEnabled)
{
    /* Deactivate buzzer when disabled */
    if(!buzzerEnabled)
    {
        setBuzzerState(false);
    }

    _buzzerEnabled = buzzerEnabled;
}

void DeviceSettings::setBuzzerState(const bool state)
{
    if(_buzzerEnabled)
    {
        if(state)
        {
            digitalWrite(DIGITAL_OUT_BUZZER_ALARM_PIN, HIGH);
        }
        else
        {
            digitalWrite(DIGITAL_OUT_BUZZER_ALARM_PIN, LOW);
        }

        _buzzerState = state;
    }
}

void DeviceSettings::setAlarmLedState(const bool state)
{
    if(state)
    {
        digitalWrite(DIGITAL_OUT_LED_ALARM_PIN, LOW);
    }
    else
    {
        digitalWrite(DIGITAL_OUT_LED_ALARM_PIN, HIGH);
    }

    _alarmLedState = state;
}

void DeviceSettings::setStatusLedState(const bool state)
{
    if(state)
    {
        digitalWrite(DIGITAL_OUT_LED_PWR_ON_PIN, LOW);
    }
    else
    {
        digitalWrite(DIGITAL_OUT_LED_PWR_ON_PIN, HIGH);
    }

    _statusLedState = state;
}

void DeviceSettings::setSensorPowerState(const bool state)
{
    if(state)
    {
        digitalWrite(DIGITAL_OUT_SENSORS_VCC_PIN, HIGH);
    }
    else
    {
        digitalWrite(DIGITAL_OUT_SENSORS_VCC_PIN, LOW);
    }

    _sensorPwrState = state;
}

bool DeviceSettings::getBuzzerState() const
{
    return _buzzerState;
}

bool DeviceSettings::getAlarmLedState() const
{
    return _alarmLedState;
}

bool DeviceSettings::getStatusLedState() const
{
    return _statusLedState;
}

bool DeviceSettings::getSwitchState() const
{
    return digitalRead(DIGITAL_IN_SWITCH_PIN);
}

bool DeviceSettings::getSensorPowerState() const
{
    return _sensorPwrState;
}
