#ifndef BLEGRILL_H
#define BLEGRILL_H

#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <Arduino.h>

#include <lib_aci.h>
#include <aci_setup.h>
#include <SPI.h>

#include "constant.h"
#include "Timer.h"
#include "services.h"
#include "BLEGrill_nRF8001.h"
#include "TempSensor.h"
#include "DeviceSettings.h"
#include "AlarmHandle.h"

class BLEGrill
{
public:
    static BLEGrill *instance();

    void setBLEBoard(const BLE* BLE_board);
    void init();
    void loop();
    void handleAciCmds(aci_state_t *aci_state, aci_evt_t *aci_evt);
    TempSensor *getSensorByNb(uint8_t sensorNb);

    void triggerMeasurement();
    void triggerNotifications();
    void switchTriggered();
    void receivedDataFromPipe(uint8_t *bytes, uint8_t byteCount, uint8_t pipe);

private:

    /* define a structure with bit fields */
    enum HARDWARE_STATES{
      hw_states_buzzerEnabled = 0,
      hw_states_buzzerState,
      hw_states_alarmLedState,
      hw_states_statusLedState,
    };

    struct ALL_TEMP_BROADCAST{
        uint8_t unit;
        uint16_t temp1;
        uint16_t temp2;
        uint16_t temp3;
        uint16_t temp4;
    };

    static BLEGrill* _instance;
    BLE _BLE_board;         /* Bluetooth Low Energy (BLE) */
    AlarmHandle _alarmHandle;
    Timer _measureTimer;    /* Timer for Reading data */
    Timer _notifyTimer;     /* Timer for sending BLE notifications */
    int _timerIdMeasure;
    int _timerIdNotify;
    uint16_t _measureIntervall;
    uint16_t _notifyIntervall;
    TempSensor* tempSensors[4];
    DeviceSettings* _deviceSets;
    bool _first_measure_pending;

    BLEGrill();
    void checkAlarms();
    void quittAlarm(const uint8_t state);
    void updateBluetoothReadPipes();
    void sendBluetoothNotifications();
    void sendAlarmViaBluetooth(const bool isAlarmActive);
    void activateBroadcast();
    void setNotifyIntervall(uint16_t *intervall);
    void setMeasureIntervall(uint16_t *intervall);
    void createSensors();
    void setAlarmSettings(const uint8_t sensorNb, const uint8_t *bytes);
    void getAlarmSettings(const TempSensor *sensor, uint8_t *bytes, uint8_t *size) const;
    void setSensorConfig(const uint8_t sensorNb, const uint8_t *bytes);
    void getSensorConfig(const TempSensor *sensor, uint8_t *bytes, uint8_t *size) const;
    void setHardwareStates(const uint8_t *bytes);
    void getHardwareStates(uint8_t *bytes, uint8_t *size) const;

};

#endif // BLEGRILL_H
