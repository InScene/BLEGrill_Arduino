/*

Copyright (c) 2012, 2013 RedBearLab

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/

#ifndef _BLEGRILL_NRF8001_H
#define _BLEGRILL_NRF8001_H

#include <lib_aci.h>
#include <aci_setup.h>

/* Include the services_lock.h to put the setup in the OTP memory of the nRF8001.
This would mean that the setup cannot be changed once put in.
However this removes the need to do the setup of the nRF8001 on every reset.*/

typedef void (*ACIPostEventHandler)(aci_state_t *aci_state, aci_evt_t *aci_evt);

// Class Definition
class BLE {

  public:

#define ADVERTISING_INTERVAL 510  // (multiple of 0.625ms)
#define ADVERTISING_TIMEOUT 0 // sec (0 means never)

    BLE(ACIPostEventHandler handlerFn);

    // Set LOCAL pipe value
    void setValueForCharacteristic(uint8_t pipe, float value);
    void setValueForCharacteristic(uint8_t pipe, uint8_t value);
    void setValueForCharacteristic(uint8_t pipe, int value);
    void setValueForCharacteristic(uint8_t pipe, uint8_t *valueSrc, uint8_t size);

    // Transmit value to BLE master
    boolean notifyClientOfValueForCharacteristic(uint8_t pipe, float value);
    boolean notifyClientOfValueForCharacteristic(uint8_t pipe, uint8_t value);
    boolean notifyClientOfValueForCharacteristic(uint8_t pipe, int value);
    boolean notifyClientOfValueForCharacteristic(uint8_t pipe, uint8_t* valueSrc, uint8_t size);

    void ble_setup(void);
    void ble_loop(void);

    byte _aci_cmd_pending;
    byte _data_credit_pending;
    boolean timing_change_done;
    boolean _isConnected;
    volatile boolean loopingSinceLastBark;

  private:

    void processACIEvent(aci_state_t *aci_state, aci_evt_t *aci_evt);
    void waitForACIResponse();
    void waitForDataCredit();
    boolean writeBufferToPipe(uint8_t *buffer, uint8_t byteCount, uint8_t pipe);
};

void setACIPostEventHandler(ACIPostEventHandler handlerFn);

#endif

