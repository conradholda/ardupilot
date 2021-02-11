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

#include <AP_HAL/AP_HAL.h>
#include "AP_PressureSensorArray.h"
#include "AP_PressureSensorArray_Backend.h"

extern const AP_HAL::HAL &hal;

/*
 * base class constructor. This initializes too.
 */
AP_PressureSensorArray_Backend::AP_PressureSensorArray_Backend(AP_PressureSensorArray::PressureSensorArray_State &_state) :
    state(_state)
    {}
