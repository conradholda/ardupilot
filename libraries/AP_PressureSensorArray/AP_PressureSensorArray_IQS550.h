/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include "AP_PressureSensorArray.h"
#include "AP_PressureSensorArray_Backend.h"
#include <AP_HAL/I2CDevice.h>

#define IQS550_ADDR     0x74
#define ACK_RESET       0x80
#define END_WINDOW      (uint16_t)0xEEEE

#define NUMBER_OF_RX    10
#define NUMBER_OF_TX    15
#define SENSOR_ELEMENTS 150

class AP_PressureSensorArray_IQS550 : public AP_PressureSensorArray_Backend
{
public:
    // static detection function
    static AP_PressureSensorArray_Backend *detect(AP_PressureSensorArray::PressureSensorArray_State &_state,
                                                  AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    uint16_t read_rx_tx(uint8_t rx, uint8_t tx) override;

    // update state
    void update_backend(void) override;

private:

    AP_PressureSensorArray_IQS550(AP_PressureSensorArray::PressureSensorArray_State &_state,
                                      AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    // constants for azoteq chip            1, 2, 3, 4, 5, 6, 7, 8, 9, 10
    uint8_t _ati_comp_tx0[NUMBER_OF_RX]  = {50,50,50,50,50,50,50,50,50,50};
    uint8_t _ati_comp_tx1[NUMBER_OF_RX]  = {50,50,50,50,50,50,50,50,50,50};
    uint8_t _ati_comp_tx2[NUMBER_OF_RX]  = {50,50,50,50,50,50,50,50,50,50};
    uint8_t _ati_comp_tx3[NUMBER_OF_RX]  = {50,50,50,50,50,50,50,50,50,50};
    uint8_t _ati_comp_tx4[NUMBER_OF_RX]  = {50,50,50,50,50,50,50,50,50,50};
    uint8_t _ati_comp_tx5[NUMBER_OF_RX]  = {50,50,50,50,50,50,50,50,50,50};
    uint8_t _ati_comp_tx6[NUMBER_OF_RX]  = {50,50,50,50,50,50,50,50,50,50};
    uint8_t _ati_comp_tx7[NUMBER_OF_RX]  = {50,50,50,50,50,50,50,50,50,50};
    uint8_t _ati_comp_tx8[NUMBER_OF_RX]  = {50,50,50,50,50,50,50,50,50,50};
    uint8_t _ati_comp_tx9[NUMBER_OF_RX]  = {50,50,50,50,50,50,50,50,50,50};
    uint8_t _ati_comp_tx10[NUMBER_OF_RX] = {50,50,50,50,50,50,50,50,50,50};
    uint8_t _ati_comp_tx11[NUMBER_OF_RX] = {50,50,50,50,50,50,50,50,50,50};
    uint8_t _ati_comp_tx12[NUMBER_OF_RX] = {50,50,50,50,50,50,50,50,50,50};
    uint8_t _ati_comp_tx13[NUMBER_OF_RX] = {50,50,50,50,50,50,50,50,50,50};
    uint8_t _ati_comp_tx14[NUMBER_OF_RX] = {50,50,50,50,50,50,50,50,50,50};

    uint8_t _ati_c_ind_tx0[NUMBER_OF_RX]  = {0,0,0,0,0,0,0,0,0,0};
    uint8_t _ati_c_ind_tx1[NUMBER_OF_RX]  = {0,0,0,0,0,0,0,0,0,0};
    uint8_t _ati_c_ind_tx2[NUMBER_OF_RX]  = {0,0,0,0,0,0,0,0,0,0};
    uint8_t _ati_c_ind_tx3[NUMBER_OF_RX]  = {0,0,0,0,0,0,0,0,0,0};
    uint8_t _ati_c_ind_tx4[NUMBER_OF_RX]  = {0,0,0,0,0,0,0,0,0,0};
    uint8_t _ati_c_ind_tx5[NUMBER_OF_RX]  = {0,0,0,0,0,0,0,0,0,0};
    uint8_t _ati_c_ind_tx6[NUMBER_OF_RX]  = {0,0,0,0,0,0,0,0,0,0};
    uint8_t _ati_c_ind_tx7[NUMBER_OF_RX]  = {0,0,0,0,0,0,0,0,0,0};
    uint8_t _ati_c_ind_tx8[NUMBER_OF_RX]  = {0,0,0,0,0,0,0,0,0,0};
    uint8_t _ati_c_ind_tx9[NUMBER_OF_RX]  = {0,0,0,0,0,0,0,0,0,0};
    uint8_t _ati_c_ind_tx10[NUMBER_OF_RX] = {0,0,0,0,0,0,0,0,0,0};
    uint8_t _ati_c_ind_tx11[NUMBER_OF_RX] = {0,0,0,0,0,0,0,0,0,0};
    uint8_t _ati_c_ind_tx12[NUMBER_OF_RX] = {0,0,0,0,0,0,0,0,0,0};
    uint8_t _ati_c_ind_tx13[NUMBER_OF_RX] = {0,0,0,0,0,0,0,0,0,0};
    uint8_t _ati_c_ind_tx14[NUMBER_OF_RX] = {0,0,0,0,0,0,0,0,0,0};

    uint8_t _ati_c_global = 15;

    enum regAddr : uint16_t
    {
        PRODUCT_NUMBER                          = 0x0000,   // 2 bytes
        PROJECT_NUMBER                          = 0x0002,   // 2 bytes
        MAJOR_VERSION                           = 0x0004,
        MINOR_VERSION                           = 0x0005,
        BOOTLOADER_STATUS                       = 0x0006,
        PREVIOUS_CYCLE_TIME                     = 0x000C,
        COUNT_VALUES                            = 0x0095,   // 300 bytes
        DELTA_VALUES                            = 0x01C1,   // 300 bytes
        REFERENCE_VALUES                        = 0x0303,   // 300 bytes
        SYSTEM_CONTROL0                         = 0x0431,
        ATI_COMPENSATION                        = 0x043F,   // 150 bytes
        ATI_C_INDIVIDUAL_ADJUST                 = 0x04D5,   // 150 bytes
        GLOBAL_ATI_C                            = 0x056B,
        ATI_TARGET                              = 0x056D,   // 2 bytes
        ATI_MAX_COUNT_LIMIT                     = 0x0575,   // 2 bytes
        REPORT_RATE_ACTIVE_MODE                 = 0x057A,   // 2 bytes
        TIMEOUT_ACTIVE_MODE                     = 0x0584,
        REFERENCE_UPDATE_TIME                   = 0x0588,
        TOTAL_RX                                = 0x063D,
        TOTAL_TX                                = 0x063E,
        RX_MAPPING                              = 0x063F,   // 10 bytes
        TX_MAPPING                              = 0x0649,   // 15 bytes
    };

    uint16_t rawValues[SENSOR_ELEMENTS];
    float pressureValues[SENSOR_ELEMENTS];

    bool init();
    void timer();

    // check sensor id and version
    bool acknowledge_reset_IQS550(void);
    bool check_version(void);

    bool get_reading(uint16_t &reading);
    bool get_reading_for_tx(uint16_t &reading, uint16_t tx);
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev;

    bool read_register(uint16_t reg, uint8_t &value) WARN_IF_UNUSED;
    bool read_register16(uint16_t reg, uint16_t &value) WARN_IF_UNUSED;
    bool read_registers16(uint16_t reg, uint16_t &value, uint8_t num_registers) WARN_IF_UNUSED;
    bool write_register(uint16_t reg, uint8_t value) WARN_IF_UNUSED;
    bool dataReady(void);
    bool Close_Comms(void);
};
