#ifndef CONSTANT
#define CONSTANT

//#define ENABLE_STARTUP_DELAY 1 /* Enables a 3 second delay on startup */
#define ENABLE_NEXTION_HMI 1

/* Enable debug prints */
//#define ACI_DEBUG 1 /* ACI lib */
//#define STATEMACHINE_DEBUG 1 /* statemachine trigger */
//#define ACTIONS_DEBUG 1 /* update read pipes, send notifications */
//#define SENSOR_DEBUG 1 /* sensor */
//#define ALARM_DEBUG 1 /* alarm  */
//#define HW_STATES_DEBUG 1 /* hardware states */

/***************************************************
 * Pin overview and defines
 * *************************************************/
/* Pin 0: Used, Serial Rx */
/* Pin 1: Used, Serial Tx */
#define DIGITAL_OUT_SENSORS_VCC_PIN     8 /* IO or I2C SDA */
#define DIGITAL_OUT_LED_ALARM_PIN       5 /* IO or I2C SCL */
/* Pin 4: Used, nRF8001 Reset */
#define DIGITAL_OUT_BUZZER_ALARM_PIN    A1 /* IO or PWM */
/* Pin 6: Used, nRF8001 REQN */
/* Pin 7: Used, nRF8001 RDYN */
#define DIGITAL_OUT_LED_PWR_ON_PIN      A0 /* IO or AnalogInput A8*/
#define DIGITAL_IN_SWITCH_PIN           A9 /* IO or PWM or AnalogInput A9 */
/* Pin 10: Free, IO or AnalogInput A10 */
/* Pin 11: Free, IO or PWM */
/* Pin 12: Free, IO or AnalogInput A11 */
/* Pin 13: Used, LED */
/* Pin A1: Free, IO or AnalogInput A1 */
#define ANALOG_SENSOR4_PIN              A2
#define ANALOG_SENSOR3_PIN              A3
#define ANALOG_SENSOR2_PIN              A4
#define ANALOG_SENSOR1_PIN              A5

#define SWITCH_INTERRUPT_PIN            PINB5
#define SWITCH_INTERRUPT                PCINT5

#define MIN_MEASURE_INTERVALL           2   /* 2 seconds */
#define MAX_TEMP_SENSORS                4

#endif CONSTANT

