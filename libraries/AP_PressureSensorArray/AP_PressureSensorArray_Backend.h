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

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_PressureSensorArray.h"

class AP_PressureSensorArray_Backend
{
public:
    // constructor. This initializes too.
    AP_PressureSensorArray_Backend(AP_PressureSensorArray::PressureSensorArray_State &_state);

    // virtual destructor for PressureSensorArray drivers can override with
    // custom dectructor is needed.
    virtual ~AP_PressureSensorArray_Backend(void) {};

    // update state structure
    virtual void update_backend() = 0;

    virtual uint16_t read_rx_tx(uint8_t rx, uint8_t tx);

    AP_PressureSensorArray::pressAry_type_t type() const { return _backend_type; }

    // return system time of last succesful read from the sensor
    uint32_t last_reading_ms() const { return state.last_reading_ms; }

protected:

    AP_PressureSensorArray::PressureSensorArray_State &state;

    // semaphore for access to shared frontend data
    HAL_Semaphore _sem;

    AP_PressureSensorArray::pressAry_type_t _backend_type;
};
