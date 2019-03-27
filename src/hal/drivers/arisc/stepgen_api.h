#ifndef _STEPGEN_API_H
#define _STEPGEN_API_H

#include <stdlib.h>
#include "msg_api.h"




enum
{
    STEPGEN_MSG_PIN_SETUP = 0x20,
    STEPGEN_MSG_TASK_ADD,
    STEPGEN_MSG_TASK_UPDATE,
    STEPGEN_MSG_ABORT,
    STEPGEN_MSG_POS_GET,
    STEPGEN_MSG_POS_SET,
    STEPGEN_MSG_WATCHDOG_SETUP,
    STEPGEN_MSG_CNT
};




/**
 * @brief   setup GPIO pin for the selected channel
 *
 * @param   c               channel id
 * @param   type            0:step, 1:dir
 * @param   port            GPIO port number
 * @param   pin             GPIO pin number
 * @param   invert          invert pin state?
 *
 * @retval  none
 */
void stepgen_pin_setup(uint8_t c, uint8_t type, uint8_t port, uint8_t pin, uint8_t invert)
{
    u32_10_t *tx = (u32_10_t*) msg_buf;

    tx->v[0] = c;
    tx->v[1] = type;
    tx->v[2] = port;
    tx->v[3] = pin;
    tx->v[4] = invert;

    msg_send(STEPGEN_MSG_PIN_SETUP, msg_buf, 5*4, 0);
}

/**
 * @brief   add a new task for the selected channel
 *
 * @param   c               channel id
 * @param   type            0:step, 1:dir
 * @param   pulses          number of pulses (ignored for DIR task)
 * @param   pin_low_time    pin LOW state duration (in nanoseconds)
 * @param   pin_high_time   pin HIGH state duration (in nanoseconds)
 *
 * @retval  none
 */
void stepgen_task_add(uint8_t c, uint8_t type, uint32_t pulses, uint32_t pin_low_time, uint32_t pin_high_time)
{
    u32_10_t *tx = (u32_10_t*) msg_buf;

    tx->v[0] = c;
    tx->v[1] = type;
    tx->v[2] = pulses;
    tx->v[3] = pin_low_time;
    tx->v[4] = pin_high_time;

    msg_send(STEPGEN_MSG_TASK_ADD, msg_buf, 5*4, 0);
}

/**
 * @brief   update time values for the current task
 *
 * @param   c               channel id
 * @param   type            0:step, 1:dir
 * @param   pin_low_time    pin LOW state duration (in nanoseconds)
 * @param   pin_high_time   pin HIGH state duration (in nanoseconds)
 *
 * @retval  none
 */
void stepgen_task_update(uint8_t c, uint8_t type, uint32_t pin_low_time, uint32_t pin_high_time)
{
    u32_10_t *tx = (u32_10_t*) msg_buf;

    tx->v[0] = c;
    tx->v[1] = type;
    tx->v[2] = pin_low_time;
    tx->v[3] = pin_high_time;

    msg_send(STEPGEN_MSG_TASK_UPDATE, msg_buf, 4*4, 0);
}

/**
 * @brief   abort all tasks for the selected channel
 * @param   c       channel id
 * @param   all     abort all task?
 * @retval  none
 */
void stepgen_abort(uint8_t c, uint8_t all)
{
    u32_10_t *tx = (u32_10_t*) msg_buf;

    tx->v[0] = c;
    tx->v[1] = all;

    msg_send(STEPGEN_MSG_ABORT, msg_buf, 2*4, 0);
}

/**
 * @brief   set channel steps position
 * @param   c   channel id
 * @retval  integer 4-bytes
 */
int32_t stepgen_pos_get(uint8_t c)
{
    u32_10_t *tx = (u32_10_t*) msg_buf;

    tx->v[0] = c;

    msg_send(STEPGEN_MSG_POS_GET, msg_buf, 1*4, 0);

    // finite loop, only 999999 tries to read an answer
    uint32_t n = 0;
    for ( n = 999999; n--; )
    {
        if ( msg_read(STEPGEN_MSG_POS_GET, msg_buf, 0) < 0 ) continue;
        else return (int32_t)tx->v[0];
    }

    return 0;
}

/**
 * @brief   set channel steps position
 * @param   c       channel id
 * @param   pos     integer 4-bytes
 * @retval  none
 */
void stepgen_pos_set(uint8_t c, int32_t pos)
{
    u32_10_t *tx = (u32_10_t*) msg_buf;

    tx->v[0] = c;
    tx->v[1] = (uint32_t)pos;

    msg_send(STEPGEN_MSG_POS_SET, msg_buf, 2*4, 0);
}

/**
 * @brief   enable/disable `abort all` watchdog
 * @param   enable      0 = disable watchdog, other values - enable watchdog
 * @param   time        watchdog wait time (in nanoseconds)
 * @retval  none
 */
void stepgen_watchdog_setup(uint8_t enable, uint32_t time)
{
    u32_10_t *tx = (u32_10_t*) msg_buf;

    tx->v[0] = enable;
    tx->v[1] = time;

    msg_send(STEPGEN_MSG_WATCHDOG_SETUP, msg_buf, 2*4, 0);
}




#endif
