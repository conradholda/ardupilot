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
#include "AP_PressureSensorArray_IQS550.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <stdio.h>

extern const AP_HAL::HAL &hal;

/*
   the constructor also initializes the pressure sensor array.
*/
AP_PressureSensorArray_IQS550::AP_PressureSensorArray_IQS550(AP_PressureSensorArray::PressureSensorArray_State &_state, AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev) :
                AP_PressureSensorArray_Backend(_state),
                dev(std::move(_dev))
{
    _backend_type = AP_PressureSensorArray::PressureSensorArray_Type_IQS550;
}


/*
   detect if a IQS550 pressure sensor array is connected. We'll detect by
   trying to take a reading on I2C. If we get a result the sensor is
   there.
*/
AP_PressureSensorArray_Backend *AP_PressureSensorArray_IQS550::detect(AP_PressureSensorArray::PressureSensorArray_State &_state, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
{
    if (!dev) {
        return nullptr;
    }

    AP_PressureSensorArray_IQS550 *sensor
        = new AP_PressureSensorArray_IQS550(_state, std::move(dev));

    if (!sensor) {
        delete sensor;
        return nullptr;
    }

    sensor->dev->get_semaphore()->take_blocking();

    if (!sensor->check_version() || !sensor->init()) {
        sensor->dev->get_semaphore()->give();
        delete sensor;
        return nullptr;
    }

    sensor->dev->get_semaphore()->give();

    return sensor;
}

bool AP_PressureSensorArray_IQS550::acknowledge_reset_IQS550(void)
{
    static uint8_t System_ctrl_0 = ACK_RESET;

    return write_register(SYSTEM_CONTROL0, System_ctrl_0);
}

bool AP_PressureSensorArray_IQS550::check_version(void)
{
    uint16_t product_num, project_num;
    if (!(read_register16(PRODUCT_NUMBER,product_num) &&
          read_register16(PROJECT_NUMBER,project_num))) {
        return false;
    }

    if ((product_num != 40) ||
        (project_num != 15)) {
        return false;
    }
    printf("Detected IQS550 on bus 0x%x\n", dev->get_bus_id());
    printf("Using product number %d with project number %d\n",product_num,project_num);

    return true;
}

// initalize sensor
bool AP_PressureSensorArray_IQS550::init()
{
    if (!(acknowledge_reset_IQS550() &&
          check_version() &&
          Close_Comms()
          ))
    {
        return false;
    }
    dev->register_periodic_callback(50000,
                                    FUNCTOR_BIND_MEMBER(&AP_PressureSensorArray_IQS550::timer, void));

    return true;
}

bool AP_PressureSensorArray_IQS550::get_reading(uint16_t &reading)
{
    if(!(read_registers16(COUNT_VALUES, reading, SENSOR_ELEMENTS) &&
         Close_Comms())) {
        return false;
    }

    return true;
}

bool AP_PressureSensorArray_IQS550::get_reading_for_tx(uint16_t &reading, uint16_t tx)
{
    uint16_t offset = NUMBER_OF_RX*tx;

    if(!(read_registers16(COUNT_VALUES+offset, reading, NUMBER_OF_RX) &&
         Close_Comms())) {
        return false;
    }

    return true;
}

bool AP_PressureSensorArray_IQS550::read_register(uint16_t reg, uint8_t &value)
{
    uint8_t b[2] = { uint8_t(reg >> 8), uint8_t(reg & 0xFF) };
    return dev->transfer(b, 2, &value, 1);
}

bool AP_PressureSensorArray_IQS550::read_register16(uint16_t reg, uint16_t &value)
{
    uint16_t v = 0;
    uint8_t b[2] = { uint8_t(reg >> 8), uint8_t(reg & 0xFF) };
    if (!dev->transfer(b, 2, (uint8_t *)&v, 2)) {
        return false;
    }
    value = be16toh(v);
    return true;
}

bool AP_PressureSensorArray_IQS550::read_registers16(uint16_t reg, uint16_t &value, uint8_t num_registers)
{
    for(int index=0; index < NUMBER_OF_RX; index++) {
        if (!read_register16(reg+sizeof(uint16_t)*index, *(&value+index))) {
            return false;
        }
    }

    return true;
}

bool AP_PressureSensorArray_IQS550::write_register(uint16_t reg, uint8_t value)
{
    uint8_t b[3] = { uint8_t(reg >> 8), uint8_t(reg & 0xFF), value };
    return dev->transfer(b, 3, nullptr, 0);
}

// the IQS550 chip wont go back to work until anything is written to END_WINDOW
bool AP_PressureSensorArray_IQS550::Close_Comms(void)
{
    uint8_t ui8DataBuffer = 15; // random number
    return write_register(END_WINDOW, ui8DataBuffer);
}

// Read the raw value
uint16_t AP_PressureSensorArray_IQS550::read_rx_tx(uint8_t rx, uint8_t tx)
{
    return state.raw_values_array[NUMBER_OF_RX*tx + rx];
}


/*
  timer called at 20Hz
*/
void AP_PressureSensorArray_IQS550::timer(void)
{
    uint16_t tx_col = 11;
    uint16_t tx11[NUMBER_OF_RX];
    if (get_reading_for_tx(*tx11, tx_col)) {
        WITH_SEMAPHORE(_sem);
        for (uint16_t index = 0; index < NUMBER_OF_RX; index++) {
            rawValues[NUMBER_OF_RX*tx_col+index] = tx11[index];
            pressureValues[NUMBER_OF_RX*tx_col+index] = 0.0f;
        }
    }
}

/*
   update the state of the sensor
*/
void AP_PressureSensorArray_IQS550::update_backend(void)
{
    WITH_SEMAPHORE(_sem);
    for (int index = 0;index < SENSOR_ELEMENTS;index++) {
        state.raw_values_array[index] = rawValues[index];
        state.pressures_array[index] = pressureValues[index];
    }
    state.last_reading_ms = AP_HAL::millis();
}
