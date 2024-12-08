/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>

#include "Semaphores.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

extern const AP_HAL::HAL& hal;

using namespace ESP32;

Semaphore::Semaphore()
{
    handle = xSemaphoreCreateRecursiveMutex();
    count = 0;
}

bool Semaphore::give()
{
    bool ok = xSemaphoreGiveRecursive(handle);
    if (ok) {
        count--;
        if (count > 0) {
            // printf("Semaphore: give: count %d\n", count);
        }
    }
    return ok;
}

bool Semaphore::take(uint32_t timeout_ms)
{
    if (timeout_ms == HAL_SEMAPHORE_BLOCK_FOREVER) {
        //! @todo we are taking a blocking semaphore, but it may not
        //  have been given, so why do we always return true?
        take_blocking();
        return true;
    }
    if (take_nonblocking()) {
        return true;
    }
    uint64_t start = AP_HAL::micros64();
    do {
        hal.scheduler->delay_microseconds(200);
        if (take_nonblocking()) {
            return true;
        }
    } while ((AP_HAL::micros64() - start) < timeout_ms*1000);
    return false;
}

void Semaphore::take_blocking()
{
    bool ok = xSemaphoreTakeRecursive(handle, portMAX_DELAY);
    if (ok) {
        count++;
        if (count > 1) {
            // printf("Semaphore: take_blocking: count %d\n", count);
        }
    }
    else {
        // printf("Semaphore: failed to take_blocking: count %d\n", count);
    }
}

bool Semaphore::take_nonblocking()
{
    bool ok = xSemaphoreTakeRecursive(handle, 0) == pdTRUE;
    if (ok) {
        count++;
        give();
    }

    //! @todo appears that we never take a non-blocking semaphore
    // printf("Semaphore: take_nonblocking: count %d\n", count);
    return ok;
}

bool Semaphore::check_owner()
{
    return xSemaphoreGetMutexHolder(handle) == xTaskGetCurrentTaskHandle();
}


/*
  BinarySemaphore implementation
 */
BinarySemaphore::BinarySemaphore(bool initial_state)
{
    _sem = xSemaphoreCreateBinary();
    if (initial_state) {
        xSemaphoreGive(_sem);
    }
}

bool BinarySemaphore::wait(uint32_t timeout_us)
{
    TickType_t ticks = pdMS_TO_TICKS(timeout_us / 1000U);
    return xSemaphoreTake(_sem, ticks) == pdTRUE;
}

bool BinarySemaphore::wait_blocking()
{
    return xSemaphoreTake(_sem, portMAX_DELAY) == pdTRUE;
}

void BinarySemaphore::signal()
{
    xSemaphoreGive(_sem);
}

void BinarySemaphore::signal_ISR()
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(_sem, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR_ARG(xHigherPriorityTaskWoken);
}

BinarySemaphore::~BinarySemaphore(void)
{
    if (_sem != nullptr) {
        vSemaphoreDelete(_sem);
    }
    _sem = nullptr;
}
