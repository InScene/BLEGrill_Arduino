#include "BLEGrill.h"

#define CHECK_BIT(var,pos)  ((var) & (1<<(pos)))
#define SET_BIT(var,pos)    ((var) |= (1<<(pos)))
#define CLEAR_BIT(var,pos)  ((var) &= ~(1<<(pos)))
#define TOGGLE_BIT(var,pos) ((var) ^= (1<<(pos)))

BLEGrill* BLEGrill::_instance = 0;
static const uint16_t _defMeasureIntvall = 2;    /* 2 seconds */
static const uint16_t _defNotifyIntervall = 10;  /* 10 seconds */
long _lastDebounceTime = 0;  // the last time the switch was toggled
long _debounceDelay = 1000;    // the switch debounce time; increase if the output flickers


/* External c function to define c++ function as callback */
extern "C" void c_handleAciCmds(aci_state_t *aci_state, aci_evt_t *aci_evt)
{
    BLEGrill::instance()->handleAciCmds(aci_state, aci_evt);
}
extern "C" void c_triggerMeasurement()
{
    BLEGrill::instance()->triggerMeasurement();
}
extern "C" void c_triggerNotifications()
{
    BLEGrill::instance()->triggerNotifications();
}
extern "C" void c_int3_trigger()
{
    BLEGrill::instance()->switchTriggered();
}


BLEGrill::BLEGrill():
    _BLE_board(c_handleAciCmds),
    _alarmHandle(),
    _measureTimer(),
    _notifyTimer(),
    _timerIdMeasure(-1),
    _timerIdNotify(-1),
    _measureIntervall(100),   /* 2 seconds */
    _notifyIntervall(100),   /* 10 seconds */
    _deviceSets(DeviceSettings::instance()),
    _first_measure_pending(true)
{
}

BLEGrill *BLEGrill::instance()
{
    if(_instance == 0)
    {
        _instance = new BLEGrill();
    }

    return _instance;
}

void BLEGrill::init()
{
#if defined(BLEND_MICRO_8MHZ)
  /* As the F_CPU = 8000000UL, the USB core make the PLLCSR = 0x02 */
  /* But the external xtal is 16000000Hz, so correct it here. */
  PLLCSR |= 0x10;             /* Need 16 MHz xtal */
  while (!(PLLCSR & (1<<PLOCK)));     /* wait for lock pll */
#elif defined(BLEND_MICRO_16MHZ)
    /* The CPU clock in bootloader is 8MHz, change to 16MHz for sketches to run (i.e. overclock running at 3.3v). */
    CLKPR = 0x80;
    CLKPR = 0;
#endif

  /* Enable serial debug */
  Serial.begin(57600);

  //while(!Serial) {}  /*  Wait until the serial port is available (useful only for the leonardo) */

  /* Configure Bluetooth LE support */
  _BLE_board.ble_setup();

  /* Init device hardware settings */
  _deviceSets->init();
  _deviceSets->setBuzzerEnabled(true);

  /* Create temperature sensors */
  createSensors();

  /* Start measure timer */
  setMeasureIntervall((uint16_t*)&_defMeasureIntvall);

  /* Set notify intervall and start notify timer */
  setNotifyIntervall((uint16_t*)&_defNotifyIntervall);

  /* Activate interrupt 3 at Uart TX. Uart still working */
  attachInterrupt(3, c_int3_trigger, FALLING );
  interrupts();                      // Enable interrupts

  Serial.println(F("Start work"));
}

void BLEGrill::loop()
{
    _measureTimer.update();
    _notifyTimer.update();



    /* Process any ACI commands or events */
    _BLE_board.ble_loop();
}

void BLEGrill::handleAciCmds(aci_state_t *aci_state, aci_evt_t *aci_evt)
{
    switch (aci_evt->evt_opcode) { /* Switch based on Event Op-Code */
        case ACI_EVT_DEVICE_STARTED: {  /* As soon as you reset the nRF8001 you will get an ACI Device Started Event */
            if (aci_evt->params.device_started.device_mode == ACI_DEVICE_STANDBY) {
                delay(100);  /* Need to let BLE board settle before we fill set-pipes otherwise first command will be ignored */

                /* Copy intial values into BLE board */
                this->updateBluetoothReadPipes();
            }
            break;
        }

        case ACI_EVT_CONNECTED: {
            _BLE_board.timing_change_done = false;
            _BLE_board._isConnected = true;
            break;
        }

        case ACI_EVT_DISCONNECTED: {
            _BLE_board.timing_change_done = false;
            _BLE_board._isConnected = false;
            break;
        }

        case ACI_EVT_DATA_RECEIVED: {  /* One of the writeable pipes (for a Characteristic) has received data */
            uint8_t *bytes = &aci_evt->params.data_received.rx_data.aci_data[0];
            uint8_t byteCount = aci_evt->len - 2;
            uint8_t pipe = aci_evt->params.data_received.rx_data.pipe_number;

            this->receivedDataFromPipe(bytes, byteCount, pipe);
            break;
        }

        case ACI_EVT_CMD_RSP: {  /* Acknowledgement of an ACI command */
            _BLE_board._aci_cmd_pending = false;
            break;
        }

        case ACI_EVT_DATA_CREDIT: {  /* Flow-control received the ability to make another transmission */
            _BLE_board._data_credit_pending = false;
            break;
        }

        case ACI_EVT_PIPE_ERROR: {  /* An error associated with a pipe */
            if (ACI_STATUS_ERROR_PEER_ATT_ERROR != aci_evt->params.pipe_error.error_code) {

                _BLE_board._data_credit_pending = false;  /* NOTE: Added while tracking down hang, don't want waitForDataCredit() to hang even though we got credit */
            }
            break;
        }

        case ACI_EVT_PIPE_STATUS: {
            if (lib_aci_is_pipe_available(aci_state, PIPE_UART_OVER_BTLE_UART_TX_TX) && (_BLE_board.timing_change_done == false)) {
                /* change the timing on the link as specified in the nRFgo studio -> nRF8001 conf. -> GAP.
                 Used to increase or decrease bandwidth */
                lib_aci_change_timing_GAP_PPCP();

                _BLE_board.timing_change_done = true;
            }
            else if (lib_aci_is_pipe_available(aci_state, PIPE_HEALTH_THERMOMETER_TEMPERATURE_MEASUREMENT_TX) && (_BLE_board.timing_change_done == false)) {
                /* change the timing on the link as specified in the nRFgo studio -> nRF8001 conf. -> GAP.
                 Used to increase or decrease bandwidth */
                lib_aci_change_timing_GAP_PPCP();

                _BLE_board.timing_change_done = true;
            }
            else if (lib_aci_is_pipe_available(aci_state, PIPE_ALARM_NOTIFIER_ALARM_INDICATION_TX) && (_BLE_board.timing_change_done == false)) {
                /* change the timing on the link as specified in the nRFgo studio -> nRF8001 conf. -> GAP.
                 Used to increase or decrease bandwidth */
                lib_aci_change_timing_GAP_PPCP();

                _BLE_board.timing_change_done = true;
            }
            else if (lib_aci_is_pipe_available(aci_state, PIPE_TEMPERATURE_SENSOR1_TEMPERATURE_MEASUREMENT_TX) && (_BLE_board.timing_change_done == false)) {
                /* change the timing on the link as specified in the nRFgo studio -> nRF8001 conf. -> GAP.
                 Used to increase or decrease bandwidth */
                lib_aci_change_timing_GAP_PPCP();

                _BLE_board.timing_change_done = true;
            }
            else if (lib_aci_is_pipe_available(aci_state, PIPE_TEMPERATURE_SENSOR2_TEMPERATURE_MEASUREMENT_TX) && (_BLE_board.timing_change_done == false)) {
                /* change the timing on the link as specified in the nRFgo studio -> nRF8001 conf. -> GAP.
                 Used to increase or decrease bandwidth */
                lib_aci_change_timing_GAP_PPCP();

                _BLE_board.timing_change_done = true;
            }
            else if (lib_aci_is_pipe_available(aci_state, PIPE_TEMPERATURE_SENSOR3_TEMPERATURE_MEASUREMENT_TX) && (_BLE_board.timing_change_done == false)) {
                /* change the timing on the link as specified in the nRFgo studio -> nRF8001 conf. -> GAP.
                 Used to increase or decrease bandwidth */
                lib_aci_change_timing_GAP_PPCP();

                _BLE_board.timing_change_done = true;
            }
            else if (lib_aci_is_pipe_available(aci_state, PIPE_TEMPERATURE_SENSOR4_TEMPERATURE_MEASUREMENT_TX) && (_BLE_board.timing_change_done == false)) {
                /* change the timing on the link as specified in the nRFgo studio -> nRF8001 conf. -> GAP.
                 Used to increase or decrease bandwidth */
                lib_aci_change_timing_GAP_PPCP();

                _BLE_board.timing_change_done = true;
            }
            break;
        }

        default: {
            /* Clear these flags in case of unhandled switch case
             to avoid staying in while() loop forever
             NOTE: Ideally wouldn't use catch-all but not familiar enough with all possible EVT cases
            */
            _BLE_board._aci_cmd_pending = false;
            _BLE_board._data_credit_pending = false;
            break;
        }
    }  /* end switch(aci_evt->evt_opcode); */
}

TempSensor* BLEGrill::getSensorByNb(uint8_t sensorNb)
{
    /* Check if sensor nb is correct. */
    if( (sensorNb >= 0) &&
        (sensorNb < MAX_TEMP_SENSORS) )
    {
        /* Get Sensor from array */
        return tempSensors[sensorNb];
    }
    else
    {
        return NULL;
    }
}


/* Measurement
 ----------------------------------------------------
 */
void BLEGrill::triggerMeasurement(void)
{
    /* Loop through all sensors and do measurement */
    for(int i=0; i<MAX_TEMP_SENSORS; i++)
    {
        TempSensor* sensor = getSensorByNb(i);

        if( sensor )
            sensor->measureTemperature();
    }

    /* Set current values to BLE pipes */
    updateBluetoothReadPipes();

    /* Check and signal alarms */
    checkAlarms();

    /* Activate sending broadcast */
  //  activateBroadcast();
}


void BLEGrill::triggerNotifications(void)
{
    sendBluetoothNotifications();
}

void BLEGrill::switchTriggered()
{
    if(millis() > (_lastDebounceTime + _debounceDelay) )
    {
        _lastDebounceTime = millis();
        Serial.println("Switch pressed");
        DeviceSettings::instance()->setBuzzerState(false);
    }
}



void BLEGrill::checkAlarms()
{
    bool alarmFound = false;

    alarmFound = _alarmHandle.checkTempSensorAlarms();

    /* When alarm found, signal it */
    if(alarmFound)
    {
        _deviceSets->setAlarmLedState(true);
        if(_alarmHandle.isNewAlarm())
        {
            sendAlarmViaBluetooth(true);
            _deviceSets->setBuzzerState(true);
        }
    }
    else
    {
       _deviceSets->setAlarmLedState(false);
       _deviceSets->setBuzzerState(false);
       sendAlarmViaBluetooth(false);
    }
}

void BLEGrill::quittAlarm(const uint8_t state)
{
    if(state > 0)
        _deviceSets->setBuzzerState(false);

#ifdef DEBUG
    if(state > 0)
        Serial.print("Alarm quitted");
    else
        Serial.print("Alarm not quitted");
#endif
}

void BLEGrill::updateBluetoothReadPipes()
{
    byte buffer[10]; /* Initialize buffer with biggest possible size */
    uint8_t bufferSize = 0;
    uint8_t tempBroadCast[sizeof(ALL_TEMP_BROADCAST)];
    uint8_t posTempBroadcast = 0;

    tempBroadCast[posTempBroadcast++] = 0; /* Temperatures are from type °C */

    /* Loop through all sensors and check temperature */
    for(int i=0; i<MAX_TEMP_SENSORS; i++)
    {
        TempSensor* sensor = getSensorByNb(i);

        /* Set data for all sensors */
        if( sensor )
        {
            Serial.print("Update Sensor:");
            Serial.print(i+1);
            Serial.print(", Temp: ");
            Serial.println(sensor->getTemperature(),DEC);

            /* Set temperature */
            sensor->getTempMeasurementBuffer(buffer, &bufferSize);
            _BLE_board.setValueForCharacteristic(sensor->getPipeTempSetId(), buffer, bufferSize);

            /* Set sensor config */
            getSensorConfig(sensor, buffer, &bufferSize);
            _BLE_board.setValueForCharacteristic(sensor->getPipeSensorConfigSetId(), buffer, bufferSize);

            /* Set Alarm settings */
            getAlarmSettings(sensor, buffer, &bufferSize);
            _BLE_board.setValueForCharacteristic(sensor->getPipeAlarmSettingsSetId(), buffer, bufferSize);

            if(posTempBroadcast < sizeof(tempBroadCast))
            {
                tempBroadCast[posTempBroadcast++] = sensor->getTemperature();
                tempBroadCast[posTempBroadcast++] = (sensor->getTemperature() >> 8);
            }
        }
        else
        {
            /* No temp */
            if(posTempBroadcast < sizeof(tempBroadCast))
            {
                tempBroadCast[posTempBroadcast++] = 0xFF;
                tempBroadCast[posTempBroadcast++] = 0xFF;
            }
        }

        /* Set All Temperature Broadcast Data */
        _BLE_board.setValueForCharacteristic(PIPE_TEMPERATURES_BROADCAST_ALL_TEMPERATURES_BROADCAST, (uint8_t*)&tempBroadCast, sizeof(tempBroadCast));

        /* Set Hardware states */
        getHardwareStates(buffer, &bufferSize);
        _BLE_board.setValueForCharacteristic(PIPE_DEVICE_SETTINGS_HARDWARE_STATES_SET, buffer, bufferSize);

        /* Set measure intervall */
        buffer[0] = _measureIntervall;
        buffer[1] += (_measureIntervall << 8);
        _BLE_board.setValueForCharacteristic(PIPE_DEVICE_SETTINGS_MEASUREMENT_INTERVAL_SET, buffer, 2);

        /* Set notify intervall */
        buffer[0] = _notifyIntervall;
        buffer[1] += (_notifyIntervall << 8);
        _BLE_board.setValueForCharacteristic(PIPE_DEVICE_SETTINGS_NOTIFY_INTERVAL_SET, buffer, 2);
    }

    Serial.println(F("updateBluetoothReadPipes"));
}

void BLEGrill::sendBluetoothNotifications()
{
  byte buffer[10]; /* Initialize buffer with biggest possible size */
  uint8_t bufferSize = 0;


  /* Loop through all sensors and check temperature */
  for(int i=0; i<MAX_TEMP_SENSORS; i++)
  {
      TempSensor* sensor = getSensorByNb(i);

      /* Update Sensor only when state and temperature okay */
      if( sensor &&
          (sensor->getState() == TempSensor::SENSOR_OK) &&
          (sensor->getTemperature() != TempSensor::TEMPERATURE_UNDEFINED) )
      {
          sensor->getTempMeasurementBuffer(buffer, &bufferSize);
          _BLE_board.notifyClientOfValueForCharacteristic(sensor->getPipeTxId(), buffer, bufferSize);
      }

  }

  Serial.println(F("sendBluetoothNotifications"));
}

void BLEGrill::sendAlarmViaBluetooth(const bool isAlarmActive)
{
  byte buffer = 0; /* Initialize buffer with biggest possible size */

  if(isAlarmActive)
      buffer = 1;

  _BLE_board.notifyClientOfValueForCharacteristic(PIPE_ALARM_NOTIFIER_ALARM_INDICATION_TX,
                                       &buffer,
                                       PIPE_ALARM_NOTIFIER_ALARM_INDICATION_TX_MAX_SIZE);
}


void BLEGrill::activateBroadcast()
{
    /* Should only be done once after updated Pipes */
    if(_first_measure_pending)
    {
        lib_aci_open_adv_pipe(PIPE_ALARM_NOTIFIER_ALARM_INDICATION_BROADCAST);
        lib_aci_open_adv_pipe(PIPE_TEMPERATURES_BROADCAST_ALL_TEMPERATURES_BROADCAST);
        lib_aci_broadcast(0/* indefinitely */, 0x1000 /* advertising interval 1s*/);
        _first_measure_pending = true;
        Serial.println(F("Broadcast activate"));
    }
}


void BLEGrill::setNotifyIntervall(uint16_t *intervall)
{
    /* Change only when intervall is different */
    if(*intervall != _notifyIntervall)
    {
        /* Notify could only so fast as measure is done */
        if(*intervall >= _measureIntervall)
        {
            _notifyIntervall = *intervall;

            /* Stop timer only if already running */
            if(_timerIdNotify >= 0)
            {
                _notifyTimer.stop(_timerIdNotify);
            }

            _timerIdNotify = _notifyTimer.every( (_notifyIntervall * 1000), c_triggerNotifications);

            Serial.print(F("Notify intervall set to: "));
        }
        else
        {
            Serial.print(F("Notify intervall to small, not changed: "));
        }

        Serial.println(_notifyIntervall,DEC);
    }
}

void BLEGrill::setMeasureIntervall(uint16_t *intervall)
{
    /* Change only when intervall is different */
    if(*intervall != _measureIntervall)
    {
        if(*intervall >= MIN_MEASURE_INTERVALL )
        {
            _measureIntervall = *intervall;

            /* Stop timer only if already running */
            if(_timerIdMeasure >= 0)
            {
                _measureTimer.stop(_timerIdMeasure);
            }

            _timerIdMeasure = _measureTimer.every( (_measureIntervall * 1000), c_triggerMeasurement);

            Serial.print(F("Measure intervall set to: "));
        }
        else
        {
            Serial.print(F("Measure intervall to small, not changed: "));
        }

        Serial.println(_measureIntervall,DEC);
    }
}


/* Route incoming data to the correct state variables */
void BLEGrill::receivedDataFromPipe(uint8_t *bytes, uint8_t byteCount, uint8_t pipe)
{
    switch (pipe)
    {
        case PIPE_UART_OVER_BTLE_UART_RX_RX:
        {
            if (byteCount <= PIPE_UART_OVER_BTLE_UART_RX_RX_MAX_SIZE)
            {
                float floatValue = *((float *)bytes);

                Serial.print(F("received UART RX data:" ));
                Serial.println(floatValue,DEC);
            }
            break;
        }

        /* Set alarm settings for sensor 1 */
        case PIPE_TEMPERATURE_SENSOR1_ALARM_SETTINGS_RX_ACK_AUTO:
        {
            Serial.println(F("received sensor1 alarm data" ));
            if (byteCount <= PIPE_TEMPERATURE_SENSOR1_ALARM_SETTINGS_RX_ACK_AUTO_MAX_SIZE)
            {
                Serial.print(F("received sensor1 alarm data:" ));
                Serial.print(bytes[0],HEX);
                Serial.print(bytes[1],HEX);
                Serial.print(bytes[2],HEX);
                Serial.print(bytes[3],HEX);
                Serial.print(bytes[4],HEX);
                Serial.println(bytes[5],HEX);
                setAlarmSettings(0, bytes);
            }
            break;
        }

        /* Set alarm settings for sensor 2 */
        case PIPE_TEMPERATURE_SENSOR2_ALARM_SETTINGS_RX_ACK_AUTO:
        {
            if (byteCount <= PIPE_TEMPERATURE_SENSOR2_ALARM_SETTINGS_RX_ACK_AUTO_MAX_SIZE)
            {
                setAlarmSettings(1, bytes);
            }
            break;
        }

        /* Set alarm settings for sensor 3 */
        case PIPE_TEMPERATURE_SENSOR3_ALARM_SETTINGS_RX_ACK_AUTO:
        {
            if (byteCount <= PIPE_TEMPERATURE_SENSOR3_ALARM_SETTINGS_RX_ACK_AUTO_MAX_SIZE)
            {
                setAlarmSettings(2, bytes);
            }
            break;
        }

        /* Set alarm settings for sensor 4 */
        case PIPE_TEMPERATURE_SENSOR4_ALARM_SETTINGS_RX_ACK_AUTO:
        {
            if (byteCount <= PIPE_TEMPERATURE_SENSOR4_ALARM_SETTINGS_RX_ACK_AUTO_MAX_SIZE)
            {
                setAlarmSettings(3, bytes);
            }
            break;
        }

        /* Set config for sensor 1 */
        case PIPE_TEMPERATURE_SENSOR1_SENSOR_CONFIG_RX_ACK_AUTO:
        {
            if (byteCount <= PIPE_TEMPERATURE_SENSOR1_SENSOR_CONFIG_RX_ACK_AUTO_MAX_SIZE)
            {
                setSensorConfig(0, bytes);
            }
            break;
        }

        /* Set config for sensor 2 */
        case PIPE_TEMPERATURE_SENSOR2_SENSOR_CONFIG_RX_ACK_AUTO:
        {
            if (byteCount <= PIPE_TEMPERATURE_SENSOR2_SENSOR_CONFIG_RX_ACK_AUTO_MAX_SIZE)
            {
                setSensorConfig(1, bytes);
            }
            break;
        }

        /* Set config for sensor 3 */
        case PIPE_TEMPERATURE_SENSOR3_SENSOR_CONFIG_RX_ACK_AUTO:
        {
            if (byteCount <= PIPE_TEMPERATURE_SENSOR3_SENSOR_CONFIG_RX_ACK_AUTO_MAX_SIZE)
            {
                setSensorConfig(2, bytes);
            }
            break;
        }

        /* Set config for sensor 4 */
        case PIPE_TEMPERATURE_SENSOR4_SENSOR_CONFIG_RX_ACK_AUTO:
        {
            if (byteCount <= PIPE_TEMPERATURE_SENSOR4_SENSOR_CONFIG_RX_ACK_AUTO_MAX_SIZE)
            {
                setSensorConfig(3, bytes);
            }
            break;
        }

        /* Set hardware states */
        case PIPE_DEVICE_SETTINGS_HARDWARE_STATES_RX_ACK_AUTO:
        {
            if (byteCount <= PIPE_DEVICE_SETTINGS_HARDWARE_STATES_RX_ACK_AUTO_MAX_SIZE)
            {
                setHardwareStates( bytes );
            }
            break;
        }

        /* Set measurement Intervall */
        case PIPE_DEVICE_SETTINGS_MEASUREMENT_INTERVAL_RX_ACK_AUTO:
        {
            if (byteCount <= PIPE_DEVICE_SETTINGS_MEASUREMENT_INTERVAL_RX_ACK_AUTO_MAX_SIZE)
            {
                setMeasureIntervall( (uint16_t*)bytes );
            }
            break;
        }

        /* Set notify Intervall */
        case PIPE_DEVICE_SETTINGS_NOTIFY_INTERVAL_RX_ACK_AUTO:
        {
            if (byteCount <= PIPE_DEVICE_SETTINGS_NOTIFY_INTERVAL_RX_ACK_AUTO_MAX_SIZE)
            {
                setNotifyIntervall( (uint16_t*)bytes );
            }
            break;
        }

        /* Quitt Alarm */
        case PIPE_ALARM_NOTIFIER_QUIT_ALARM_RX_ACK_AUTO:
        {
            if (byteCount <= PIPE_ALARM_NOTIFIER_QUIT_ALARM_RX_ACK_AUTO_MAX_SIZE)
            {
                quittAlarm(bytes[0]);
            }
            break;
        }
    }  /* end switch(pipe) */
}


void BLEGrill::createSensors()
{
    /* Activate temperature sensor 1 */
    tempSensors[0] = new TempSensor(ANALOG_SENSOR1_PIN,
                                    1,
                                    TempSensor::MAVERICK,
                                    PIPE_TEMPERATURE_SENSOR1_TEMPERATURE_MEASUREMENT_SET,
                                    PIPE_TEMPERATURE_SENSOR1_TEMPERATURE_MEASUREMENT_TX,
                                    PIPE_TEMPERATURE_SENSOR1_ALARM_SETTINGS_SET,
                                    PIPE_TEMPERATURE_SENSOR1_SENSOR_CONFIG_SET);
    tempSensors[1] = new TempSensor(ANALOG_SENSOR2_PIN,
                                    2,
                                    TempSensor::MAVERICK,
                                    PIPE_TEMPERATURE_SENSOR2_TEMPERATURE_MEASUREMENT_SET,
                                    PIPE_TEMPERATURE_SENSOR2_TEMPERATURE_MEASUREMENT_TX,
                                    PIPE_TEMPERATURE_SENSOR2_ALARM_SETTINGS_SET,
                                    PIPE_TEMPERATURE_SENSOR2_SENSOR_CONFIG_SET);
    tempSensors[2] = new TempSensor(ANALOG_SENSOR3_PIN,
                                    3,
                                    TempSensor::MAVERICK,
                                    PIPE_TEMPERATURE_SENSOR3_TEMPERATURE_MEASUREMENT_SET,
                                    PIPE_TEMPERATURE_SENSOR3_TEMPERATURE_MEASUREMENT_TX,
                                    PIPE_TEMPERATURE_SENSOR3_ALARM_SETTINGS_SET,
                                    PIPE_TEMPERATURE_SENSOR3_SENSOR_CONFIG_SET);
    tempSensors[3] = new TempSensor(ANALOG_SENSOR4_PIN,
                                    4,
                                    TempSensor::MAVERICK,
                                    PIPE_TEMPERATURE_SENSOR4_TEMPERATURE_MEASUREMENT_SET,
                                    PIPE_TEMPERATURE_SENSOR4_TEMPERATURE_MEASUREMENT_TX,
                                    PIPE_TEMPERATURE_SENSOR4_ALARM_SETTINGS_SET,
                                    PIPE_TEMPERATURE_SENSOR4_SENSOR_CONFIG_SET);

    /* Activate sensors and set default values */
    for(int i=0; i<MAX_TEMP_SENSORS; i++)
    {
        tempSensors[i]->activateSensor(true);
        tempSensors[i]->setHighTempBorder(3000);
        tempSensors[i]->setLowTempBorder(2000);
        tempSensors[i]->setAlarmType(TempSensor::HIGH_LOW_ALARM);
    }

    // TODO: Remove, only for developing
    tempSensors[2]->activateSensor(false);
    tempSensors[3]->activateSensor(false);
}

void BLEGrill::setSensorConfig(const uint8_t sensorNb, const uint8_t *bytes)
{
    TempSensor* sensor = getSensorByNb(sensorNb);
    if(sensor)
    {
        bool sensorEnable = false;
        if(bytes[0] != 0)
            sensorEnable = true;

        uint8_t sensorType = bytes[1];

        sensor->activateSensor(sensorEnable);
        sensor->setSensorType(sensorType);

#ifdef DEBUG
        Serial.print("Set sensor config sensor: ");
        Serial.print(sensor->getSensorNb(),DEC);
        Serial.print(", enable:");
        Serial.print(sensorEnable,DEC);
        Serial.print(", type:");
        Serial.println(sensorType,DEC);
#endif
    }
    else
    {
#ifdef DEBUG
        Serial.print("Sensor config not set, wrong sensor:");
        Serial.println(sensorNb,DEC);
#endif
    }
}

void BLEGrill::getSensorConfig(const TempSensor* sensor, uint8_t *bytes, uint8_t *size) const
{
    if(sensor->sensorIsActive())
        bytes[0] = 1;
    else
        bytes[0] = 0;
    bytes[1] = sensor->getSensorType();

    *size = 2;

#ifdef DEBUG
        Serial.print("Sensor config Sensor:");
        Serial.print(sensor->getSensorNb(),DEC);
        Serial.print(", data:");
        Serial.print(bytes[0],HEX);
        Serial.print(",");
        Serial.println(bytes[1],HEX);
#endif
}

void BLEGrill::setAlarmSettings(const uint8_t sensorNb, const uint8_t *bytes)
{
    TempSensor* sensor = getSensorByNb(sensorNb);
    if(sensor)
    {
        /* uint8_t alarmState = bytes[0]; */ /* Alarm state readonly, so ignore this byte */
        uint8_t alarmType = bytes[1];

        uint16_t highTempBorder = bytes[2];
        highTempBorder += (bytes[3] << 8);

        uint16_t lowTempBorder = bytes[4];
        lowTempBorder += (bytes[5] << 8);


        sensor->setAlarmType(alarmType);
        sensor->setHighTempBorder(highTempBorder);
        sensor->setLowTempBorder(lowTempBorder);

#ifdef DEBUG
        Serial.print("Set Alarm Settings Sensor:");
        Serial.print(sensor->getSensorNb(),DEC);
        Serial.print(", alarm type:");
        Serial.print(alarmType,DEC);
        Serial.print(", highTemp:");
        Serial.print(highTempBorder,DEC);
        Serial.print(", lowTemp");
        Serial.println(lowTempBorder,DEC);
#endif
    }
    else
    {
#ifdef DEBUG
        Serial.print("Sensor alarm settings not set, wrong sensor:");
        Serial.println(sensorNb,DEC);
#endif
    }

}

void BLEGrill::getAlarmSettings(const TempSensor* sensor, uint8_t *bytes, uint8_t *size) const
{
    bytes[0] = sensor->getAlarmState();

    bytes[1] = sensor->getAlarmType();

    bytes[2] = sensor->getHighTempBorder();
    bytes[3] = (sensor->getHighTempBorder() >> 8);

    bytes[4] = sensor->getLowTempBorder();
    bytes[5] = (sensor->getLowTempBorder() >> 8);

    *size = 6;

#ifdef DEBUG
        Serial.print("Alarm Settings Sensor:");
        Serial.print(sensor->getSensorNb(),DEC);
        Serial.print(", data:");
        Serial.print(bytes[0],HEX);
        Serial.print(",");
        Serial.print(bytes[1],HEX);
        Serial.print(",");
        Serial.print(bytes[2],HEX);
        Serial.print(",");
        Serial.print(bytes[3],HEX);
        Serial.print(",");
        Serial.print(bytes[4],HEX);
        Serial.print(",");
        Serial.println(bytes[5],HEX);
#endif
}

void BLEGrill::setHardwareStates(const uint8_t *bytes)
{
    bool buzzerEnable = false;
    bool buzzerState = false;
    bool alarmLedState = false;
    bool statusLedState = false;

    if( CHECK_BIT(bytes[0], hw_states_buzzerEnabled) )
        buzzerEnable = true;
    if( CHECK_BIT(bytes[0], hw_states_buzzerState) )
        buzzerState = true;
    if( CHECK_BIT(bytes[0], hw_states_alarmLedState) )
        alarmLedState = true;
    if( CHECK_BIT(bytes[0], hw_states_statusLedState) )
        statusLedState = true;

    _deviceSets->setBuzzerEnabled(buzzerEnable);
    _deviceSets->setBuzzerState(buzzerState);
    _deviceSets->setAlarmLedState(alarmLedState);
    _deviceSets->setStatusLedState(statusLedState);

#ifdef DEBUG
        Serial.print("Set hadware stats. BuzzerEnable:");
        Serial.print(buzzerEnable,DEC);
        Serial.print(", buzzerState:");
        Serial.print(buzzerState,DEC);
        Serial.print(", alarmLedState:");
        Serial.print(alarmLedState,DEC);
        Serial.print(", statusLedState:");
        Serial.println(statusLedState,DEC);
#endif
}

void BLEGrill::getHardwareStates(uint8_t *bytes, uint8_t *size) const
{
    uint8_t hwStates = 0;

    if(_deviceSets->getBuzzerEnabled())
        SET_BIT(hwStates, hw_states_buzzerEnabled);
    if(_deviceSets->getBuzzerState())
        SET_BIT(hwStates, hw_states_buzzerState);
    if(_deviceSets->getAlarmLedState())
        SET_BIT(hwStates, hw_states_alarmLedState);
    if(_deviceSets->getStatusLedState())
        SET_BIT(hwStates, hw_states_statusLedState);

    bytes[0] = hwStates;
    *size = 1;

#ifdef DEBUG
        Serial.print("HW States:");
        Serial.println(hwStates,BIN);
#endif
}
