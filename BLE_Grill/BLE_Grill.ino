/*

Copyright (c) 2012, 2013 RedBearLab

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/

/* Includes added here although they are not used in this file
 * because otherwise the Arduino sketch compile doesn't compile */
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <Arduino.h>

#include <lib_aci.h>
#include <aci_setup.h>
#include <SPI.h>
#include "Timer.h"
#include "services.h"
#include "BLEGrill_nRF8001.h"
#include "BLEGrill.h"


/* Main Application */
BLEGrill* _bleGrill = BLEGrill::instance();

/* W.D. interrupt handler should be as short as possible, so it sets
   watchdogWokeUp = true to indicate to run-loop that watchdog timer fired
*/
volatile boolean watchdogWokeUp = false;

void setup()
{
    /* Initialize the main application */
    _bleGrill->init();

    /* Initiate watchdog timer*/
    startWatchdogTimer();
}


void loop()
{
    /* Check whether the watchdog barked */
    if (watchdogWokeUp)
    {
      watchdogWokeUp = false;  /* Reset flag until watchdog fires again */
    }

    _bleGrill->loop();
}


/* WATCHDOG
 ----------------------------------------------------
*/
/* Enable the Watchdog timer and configure timer duration */
void startWatchdogTimer() {

  /* Clear the reset flag, the WDRF bit (bit 3) of MCUSR. */
  MCUSR = MCUSR & B11110111;

  /* Set the WDCE bit (bit 4) and the WDE bit (bit 3)
     of WDTCSR. The WDCE bit must be set in order to
     change WDE or the watchdog prescalers. Setting the
     WDCE bit will allow updtaes to the prescalers and
     WDE for 4 clock cycles then it will be reset by
     hardware.
  */
  WDTCSR = WDTCSR | B00011000;

  /* Set the watchdog timeout prescaler value to 1024 K
     which will yeild a time-out interval of about 8.0 s.
  */
//  WDTCSR = B00100001;  // ~8s
  WDTCSR = B00100000;  /* ~4s */

  /* Enable the watchdog timer interupt. */
  WDTCSR = WDTCSR | B01000000;
  MCUSR = MCUSR & B11110111;
}


/* Register function for Watchdog interrupt */
ISR(WDT_vect) {

    watchdogWokeUp = true;
    wdt_reset();  /* Pet the Watchdog, stop it from forcing hardware reset */
}
