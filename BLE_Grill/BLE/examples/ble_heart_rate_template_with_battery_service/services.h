/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
*
* The information contained herein is property of Nordic Semiconductor ASA.
* Terms and conditions of usage are described in detail in NORDIC
* SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
*
* Licensees are granted free, non-transferable use of the information. NO
* WARRANTY of ANY KIND is provided. This heading must NOT be removed from
* the file.
*/

/**
* This file is autogenerated by nRFgo Studio 1.14.0.2358  
*/

#ifndef SETUP_MESSAGES_H__
#define SETUP_MESSAGES_H__

#include "hal_platform.h" 
#include "aci.h"
#define PIPE_DEVICE_INFORMATION_SERIAL_NUMBER_STRING_SET 1
#define PIPE_DEVICE_INFORMATION_MANUFACTURER_NAME_STRING_SET 2
#define PIPE_DEVICE_INFORMATION_SYSTEM_ID_SET 3
#define PIPE_DEVICE_INFORMATION_FIRMWARE_REVISION_STRING_SET 4
#define PIPE_HEART_RATE_HEART_RATE_MEASUREMENT_TX 5
#define PIPE_HEART_RATE_HEART_RATE_SENSOR_LOCATION_SET 6
#define PIPE_HEART_RATE_HEART_RATE_CONTROL_POINT_RX_ACK 7
#define PIPE_BATTERY_BATTERY_LEVEL_TX 8
#define PIPE_BATTERY_BATTERY_LEVEL_SET 9

#define NUMBER_OF_PIPES 9

#define SERVICES_PIPE_TYPE_MAPPING_CONTENT {\
  {ACI_STORE_LOCAL, ACI_SET},   \
  {ACI_STORE_LOCAL, ACI_SET},   \
  {ACI_STORE_LOCAL, ACI_SET},   \
  {ACI_STORE_LOCAL, ACI_SET},   \
  {ACI_STORE_LOCAL, ACI_TX},   \
  {ACI_STORE_LOCAL, ACI_SET},   \
  {ACI_STORE_LOCAL, ACI_RX_ACK},   \
  {ACI_STORE_LOCAL, ACI_TX},   \
  {ACI_STORE_LOCAL, ACI_SET},   \
}

#define GAP_PPCP_MAX_CONN_INT 0x320 /**< Maximum connection interval as a multiple of 1.25 msec , 0xFFFF means no specific value requested */
#define GAP_PPCP_MIN_CONN_INT  0x190 /**< Minimum connection interval as a multiple of 1.25 msec , 0xFFFF means no specific maximum*/
#define GAP_PPCP_SLAVE_LATENCY 0
#define GAP_PPCP_CONN_TIMEOUT 0x1f6 /** Connection Supervision timeout multiplier as a multiple of 10msec, 0xFFFF means no specific value requested */

#define NB_SETUP_MESSAGES 29
#define SETUP_MESSAGES_CONTENT {\
    {0x00,\
        {\
            0x07,0x06,0x00,0x00,0x03,0x02,0x41,0xd7,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x10,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x09,0x00,0x09,0x01,0x01,0x00,0x00,0x06,0x00,0x00,\
            0x81,0x0d,0x18,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x10,0x1c,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
            0x00,0x00,0x40,0x12,0x00,0x00,0x40,0x12,0x03,0x90,0x00,0x64,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x10,0x38,0x02,0xff,0x00,0x00,0x00,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
            0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x01,0x00,0x00,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x05,0x06,0x10,0x54,0x00,0x01,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0x00,0x04,0x04,0x02,0x02,0x00,0x01,0x28,0x00,0x01,0x00,0x18,0x04,0x04,0x05,0x05,0x00,\
            0x02,0x28,0x03,0x01,0x0e,0x03,0x00,0x00,0x2a,0x04,0x14,0x0f,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0x1c,0x0f,0x00,0x03,0x2a,0x00,0x01,0x4e,0x6f,0x72,0x64,0x69,0x63,0x20,0x48,0x52,0x4d,\
            0x20,0x56,0x31,0x2e,0x30,0x04,0x04,0x05,0x05,0x00,0x04,0x28,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0x38,0x03,0x01,0x02,0x05,0x00,0x01,0x2a,0x06,0x04,0x03,0x02,0x00,0x05,0x2a,0x01,0x01,\
            0x40,0x03,0x04,0x04,0x05,0x05,0x00,0x06,0x28,0x03,0x01,0x02,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0x54,0x07,0x00,0x04,0x2a,0x06,0x04,0x09,0x08,0x00,0x07,0x2a,0x04,0x01,0x90,0x01,0x20,\
            0x03,0x00,0x00,0xf6,0x01,0x04,0x04,0x02,0x02,0x00,0x08,0x28,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0x70,0x00,0x01,0x01,0x18,0x04,0x04,0x02,0x02,0x00,0x09,0x28,0x00,0x01,0x0a,0x18,0x04,\
            0x04,0x05,0x05,0x00,0x0a,0x28,0x03,0x01,0x02,0x0b,0x00,0x25,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0x8c,0x2a,0x06,0x04,0x05,0x04,0x00,0x0b,0x2a,0x25,0x01,0x31,0x35,0x38,0x37,0x06,0x04,\
            0x08,0x07,0x00,0x0c,0x29,0x04,0x01,0x19,0x00,0x00,0x00,0x01,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0xa8,0x00,0x00,0x04,0x04,0x05,0x05,0x00,0x0d,0x28,0x03,0x01,0x02,0x0e,0x00,0x29,0x2a,\
            0x06,0x04,0x15,0x14,0x00,0x0e,0x2a,0x29,0x01,0x4e,0x6f,0x72,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0xc4,0x64,0x69,0x63,0x53,0x65,0x6d,0x69,0x63,0x6f,0x6e,0x64,0x75,0x63,0x74,0x6f,0x72,\
            0x00,0x06,0x04,0x08,0x07,0x00,0x0f,0x29,0x04,0x01,0x19,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0xe0,0x00,0x00,0x01,0x00,0x00,0x04,0x04,0x05,0x05,0x00,0x10,0x28,0x03,0x01,0x02,0x11,\
            0x00,0x23,0x2a,0x06,0x04,0x09,0x08,0x00,0x11,0x2a,0x23,0x01,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0xfc,0x55,0xaa,0x55,0xaa,0x55,0xaa,0x55,0xaa,0x04,0x04,0x05,0x05,0x00,0x12,0x28,0x03,\
            0x01,0x02,0x13,0x00,0x26,0x2a,0x06,0x04,0x05,0x04,0x00,0x13,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x21,0x18,0x2a,0x26,0x01,0x30,0x31,0x2e,0x31,0x06,0x04,0x08,0x07,0x00,0x14,0x29,0x04,0x01,\
            0x19,0x00,0x00,0x00,0x01,0x00,0x00,0x04,0x04,0x02,0x02,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x21,0x34,0x15,0x28,0x00,0x01,0x0d,0x18,0x04,0x04,0x05,0x05,0x00,0x16,0x28,0x03,0x01,0x10,\
            0x17,0x00,0x37,0x2a,0x14,0x00,0x13,0x00,0x00,0x17,0x2a,0x37,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x21,0x50,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
            0x00,0x00,0x00,0x00,0x46,0x14,0x03,0x02,0x00,0x18,0x29,0x02,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x21,0x6c,0x01,0x00,0x00,0x04,0x04,0x05,0x05,0x00,0x19,0x28,0x03,0x01,0x02,0x1a,0x00,0x38,\
            0x2a,0x06,0x04,0x02,0x01,0x00,0x1a,0x2a,0x38,0x01,0x03,0x04,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x21,0x88,0x04,0x05,0x05,0x00,0x1b,0x28,0x03,0x01,0x08,0x1c,0x00,0x39,0x2a,0x46,0x10,0x02,\
            0x01,0x00,0x1c,0x2a,0x39,0x01,0x00,0x04,0x04,0x02,0x02,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x21,0xa4,0x1d,0x28,0x00,0x01,0x0f,0x18,0x04,0x04,0x05,0x05,0x00,0x1e,0x28,0x03,0x01,0x12,\
            0x1f,0x00,0x19,0x2a,0x16,0x04,0x02,0x01,0x00,0x1f,0x2a,0x19,\
        },\
    },\
    {0x00,\
        {\
            0x11,0x06,0x21,0xc0,0x01,0x64,0x46,0x14,0x03,0x02,0x00,0x20,0x29,0x02,0x01,0x00,0x00,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x40,0x00,0x2a,0x00,0x01,0x00,0x00,0x04,0x00,0x03,0x00,0x00,0x2a,0x25,0x01,0x00,0x80,0x04,\
            0x00,0x0b,0x00,0x00,0x2a,0x29,0x01,0x00,0x80,0x04,0x00,0x0e,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x40,0x1c,0x00,0x00,0x2a,0x23,0x01,0x00,0x80,0x04,0x00,0x11,0x00,0x00,0x2a,0x26,0x01,0x00,\
            0x80,0x04,0x00,0x13,0x00,0x00,0x2a,0x37,0x01,0x00,0x02,0x04,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x40,0x38,0x00,0x17,0x00,0x18,0x2a,0x38,0x01,0x00,0x80,0x04,0x00,0x1a,0x00,0x00,0x2a,0x39,\
            0x01,0x00,0x10,0x04,0x00,0x1c,0x00,0x00,0x2a,0x19,0x01,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x09,0x06,0x40,0x54,0x82,0x04,0x00,0x1f,0x00,0x20,\
        },\
    },\
    {0x00,\
        {\
            0x1e,0x06,0x60,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
            0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x19,0x06,0x70,0x00,0x19,0x02,0x40,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
            0x00,0x00,0x00,0x00,0x00,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x06,0x06,0xf0,0x00,0x03,0x0b,0x8c,\
        },\
    },\
}

#endif
