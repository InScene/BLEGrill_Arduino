#ifndef DEVICESETTINGS_H
#define DEVICESETTINGS_H

#include <stdio.h>

class DeviceSettings
{
public:
    static DeviceSettings *instance();
    void init(void);

    bool getBuzzerEnabled() const;
    void setBuzzerEnabled(const bool getBuzzerEnabled);

    void setBuzzerState(const bool state);
    void setAlarmLedState(const bool state);
    void setStatusLedState(const bool state);
    void setSensorPowerState(const bool state);

    bool getBuzzerState() const;
    bool getAlarmLedState() const;
    bool getStatusLedState() const;
    bool getSwitchState() const;
    bool getSensorPowerState() const;

private:
    DeviceSettings();

    static DeviceSettings* _instance;
    bool _buzzerEnabled;
    bool _buzzerState;
    bool _alarmLedState;
    bool _statusLedState;
    bool _sensorPwrState;


};

#endif // DEVICESETTINGS_H
