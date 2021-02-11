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
#include "AP_PressureSensorArray.h"
#include "AP_PressureSensorArray_IQS550.h"

#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL &hal;

AP_PressureSensorArray::AP_PressureSensorArray()
{
    driver = nullptr;
    num_instances = 0;

    _singleton = this;
}

/*
 * initalize the PressureSensorArray class.
 */
void AP_PressureSensorArray::init(void)
{
    if (num_instances != 0) {
        return;
    }

    detect_instance();
    num_instances = 1;
}

/*
 * update PressureSensorArray state for the single instance. This should be
 * called by the main loop no faster than what the chip can update itself
 * (~20 Hz at time of writing).
 */
void AP_PressureSensorArray::update(void)
{
    if (driver != nullptr) {
        driver->update_backend();
    }

    Log_PRESSARY();
}

bool AP_PressureSensorArray::_add_backend(AP_PressureSensorArray_Backend *backend)
{
    if (!backend) {
        return false;
    }
    if (num_instances >= 1) {
        AP_HAL::panic("Too many PRESSURE SENSOR ARRAY backends.");
    }

    driver = backend;
    return true;
}

void AP_PressureSensorArray::detect_instance()
{
    pressAry_type_t _type = AP_PressureSensorArray::PressureSensorArray_Type_IQS550;//get_type();
    state.type = _type;
    switch (_type) {
    case PressureSensorArray_Type_IQS550:
        FOREACH_I2C(i) {
            if(AP_PressureSensorArray::_add_backend(AP_PressureSensorArray_IQS550::detect(state,hal.i2c_mgr->get_device(i,AP_PRESSURE_SENSOR_ARRAY_IQS550_ADDR)))) {
                break;
            }
        }
        break;
    default:
        break;
    }
}

AP_PressureSensorArray_Backend *AP_PressureSensorArray::get_backend(void) const {
    if (driver != nullptr) {
        if (driver->type() == PressureSensorArray_Type_None) {
            // pretend it isn't here; disabled at runtime?
            return nullptr;
        }
    }
    return driver;
};

void AP_PressureSensorArray::Log_PRESSARY()
{
//    if (_log_pressAry_bit == uint32_t(-1)) {
//        return;
//    }

//    AP_Logger &logger = AP::logger();
//    if(!logger.should_log(_log_pressAry_bit)) {
//        return;
//    }

    AP_PressureSensorArray_Backend *ary = get_backend();
    if (ary == nullptr) {

        AP::logger().Write("T3ST", "TimeUS,Alt",
                               "sm", // units: seconds, meters
                               "FB", // mult: 1e-6, 1e-2
                               "Qf", // format: uint64_t, float
                               AP_HAL::micros64(),
                               1.05);

        return;
    }

    // TODO get pitch and rot velocity to estimate aoa

    const struct log_PRESSARY pkt = {
            LOG_PACKET_HEADER_INIT(LOG_PRESSARY_MSG),
            time_us     : AP_HAL::micros64(),
            pitch       : 0.0f,            // TODO just setting 0 until pitch backend pointer is done
            p_rate      : 0.0f,            // TODO just setting 0 until rot vel backend pointer is done
            rx0         : ary->read_rx_tx(0,11) ,
            rx1         : ary->read_rx_tx(1,11) ,
            rx2         : ary->read_rx_tx(2,11) ,
            rx3         : ary->read_rx_tx(3,11) ,
            rx4         : ary->read_rx_tx(4,11) ,
            rx5         : ary->read_rx_tx(5,11) ,
            rx6         : ary->read_rx_tx(6,11) ,
            rx7         : ary->read_rx_tx(7,11) ,
            rx8         : ary->read_rx_tx(8,11) ,
            rx9         : ary->read_rx_tx(9,11) ,
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));

    AP::logger().Write("PRSA","TimeUS,RX0,RX1,RX2,RX3,RX4,RX5,RX6,RX7,RX8,RX9","Qiiiiiiiiii",
            AP_HAL::micros64(),
            (int) ary->read_rx_tx(0,11),
            (int) ary->read_rx_tx(1,11),
            (int) ary->read_rx_tx(2,11),
            (int) ary->read_rx_tx(3,11),
            (int) ary->read_rx_tx(4,11),
            (int) ary->read_rx_tx(5,11),
            (int) ary->read_rx_tx(6,11),
            (int) ary->read_rx_tx(7,11),
            (int) ary->read_rx_tx(8,11),
            (int) ary->read_rx_tx(9,11));

}

AP_PressureSensorArray *AP_PressureSensorArray::_singleton;

namespace AP {

AP_PressureSensorArray *pressuresensorarray()
{
    return AP_PressureSensorArray::get_singleton();
}

}




