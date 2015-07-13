#ifndef TEMPSENSOR_H
#define TEMPSENSOR_H

#include <stdio.h>

class TempSensor
{
public:

    enum SENSORTYPE {
        ACURITE = 0,
        FANTAST,
        ROSENSTEIN,
        MAVERICK
    };

    enum SENSORSTATE {
        SENSOR_OFF = 0,
        SENSOR_OK,
        NO_SENSOR,
        MEASUREMENT_ERR
    };

    enum SENSOR_ALARM_TYPE{
        NO_ALARM = 0,
        HIGH_ALARM,
        LOW_ALARM,
        HIGH_LOW_ALARM
    };

    enum SENSOR_ALARM_STATE{
        STATE_NO_ALARM = 0,
        STATE_HIGH_ALARM,
        STATE_LOW_ALARM,
        STATE_NO_SENSOR_ALARM,
        STATE_MEASURE_ERR_ALARM
    };

    static const uint16_t TEMPERATURE_UNDEFINED = 0xFFFF;

    TempSensor(const uint8_t pin, const uint8_t sensorNb, const SENSORTYPE type,
               uint8_t pipeTempSetId, uint8_t pipeTxId,
               uint8_t pipeAlarmSettingSetId, uint8_t pipeSensorConfigSetId);
    void activateSensor(const bool activate);
    void measureTemperature();
    void setSensorType(uint8_t type);
    void setHighTempBorder(const uint16_t tempValue);
    void setLowTempBorder(const uint16_t tempValue);
    void setAlarmType(const uint8_t alarmType);

    void getTempMeasurementBuffer(uint8_t *dest, uint8_t *size) const;
    uint8_t getSensorNb() const;
    uint16_t getTemperatureFixedPoint() const;
    void getTemperatureAsChar(char* buff) const;
    uint16_t getHighTempBorder() const;
    uint16_t getLowTempBorder() const;
    uint8_t getPipeTempSetId() const;
    uint8_t getPipeTxId() const;
    uint8_t getPipeAlarmSettingsSetId() const;
    uint8_t getPipeSensorConfigSetId() const;
    SENSORSTATE getState() const;
    bool hasAlarm() const;
    bool sensorIsActive() const;
    SENSOR_ALARM_TYPE getAlarmType() const;
    SENSORTYPE getSensorType()const ;
    SENSOR_ALARM_STATE getAlarmState() const;

private:
    static const uint8_t _TEMP_ALARM_HYSTERESIS = 2;
    uint16_t _temperature;
    uint8_t _pin;
    uint8_t _sensorNb;
    SENSORTYPE _type;
    SENSORSTATE _state;
    SENSOR_ALARM_TYPE _alarmType;
    SENSOR_ALARM_STATE _alarmState;
    bool _enabled;
    uint16_t _highTempBorder;
    uint16_t _lowTempBorder;
    uint8_t _pipeSetId;
    uint8_t _pipeTxId;
    uint8_t _pipeAlarmSettingsSetId;
    uint8_t _pipeSensorConfigSetId;
    bool _alarmIsActive;

    void doMeasurement();
    uint16_t convertValueIntoTemperature(const double Rt) const;
    bool isHighAlarm() const;
    bool isLowAlarm() const;
    void checkAlarm();
};


#endif // TEMPSENSOR_H
