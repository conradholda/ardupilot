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

#include <AP_HAL/AP_HAL.h>

#define AP_PRESSURE_SENSOR_ARRAY_IQS550_ADDR 0x74       // TODO: Add to some sort of params folder

class AP_PressureSensorArray_Backend;

class AP_PressureSensorArray
{
    friend class AP_PressureSensorArray_Backend;

public:
    AP_PressureSensorArray();

    /* Do not allow copies */
    AP_PressureSensorArray(const AP_PressureSensorArray &other) = delete;
    AP_PressureSensorArray &operator=(const AP_PressureSensorArray&) = delete;

    // Pressure Sensor Array types
    typedef enum{
        PressureSensorArray_Type_None   = 0,
        PressureSensorArray_Type_IQS550 = 1,
    } pressAry_type_t;

    struct PressureSensorArray_State {
        uint16_t            raw_values_array[150];      // array of raw values from sensor array TODO: Fix magic number
        float               pressures_array[150];       // array of pressure values in Pa from sensor array TODO: Fix magic number
        uint32_t            last_reading_ms;            // system time of last successful update from sensor
        pressAry_type_t     type;                       // the type of pressure array
    };

    // Return the number of range finder instances
    uint8_t num_sensors(void) const {
        return num_instances;
    }

    // initialise the pressure sensor array object, loading backend drivers
    void init(void);

    // update the pressure sensor array object, asking backends to push data to
    // the frontend
    void update(void);

    AP_PressureSensorArray_Backend *get_backend(void) const;

    // set type of pressure sensor array
    void set_type(pressAry_type_t type) { state.type = type; };

    // get type of pressure sensor array
    pressAry_type_t get_type(void) { return state.type; };

    // indicate which bit in LOG_BITMASK indicates pressAry should be logged
    void set_log_pressAry_bit(uint32_t bit) { _log_pressAry_bit = bit; }

    // get singleton
    static AP_PressureSensorArray *get_singleton(void) { return _singleton; }

private:
    // singleton
    static AP_PressureSensorArray *_singleton;

    PressureSensorArray_State           state;              // holds relevant data
    AP_PressureSensorArray_Backend *    driver;             // backend driver (pointer)
    uint8_t                             num_instances;      // should only be one. This was just to make sure not initializing more than once

    void detect_instance();

    bool _add_backend(AP_PressureSensorArray_Backend *backend);

    uint32_t _log_pressAry_bit = -1;
    void Log_PRESSARY();
};

namespace AP {
    AP_PressureSensorArray &pressureArray();
};
