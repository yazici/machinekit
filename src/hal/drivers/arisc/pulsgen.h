/**
 * @file    pulsgen.h
 * @brief   PULSGEN related data
 * @author  Mikhail Vydrenko (mikhail@vydrenko.ru)
 */

#ifndef _PULSGEN_H
#define _PULSGEN_H

#include "msg.h"




#define PULSGEN_CH_CNT 64  ///< maximum number of pulse generator channels

/// messages types
enum
{
    PULSGEN_MSG_PIN_SETUP = 0x20,
    PULSGEN_MSG_TASK_SETUP,
    PULSGEN_MSG_TASK_ABORT,
    PULSGEN_MSG_TASK_STATE,
    PULSGEN_MSG_TASK_TOGGLES,
    PULSGEN_MSG_WATCHDOG_SETUP
};

/// the message data access
struct pulsgen_msg_pin_setup_t { uint32_t ch; uint32_t port; uint32_t pin; uint32_t inverted; };
struct pulsgen_msg_task_setup_t { uint32_t ch; uint32_t toggles;
    uint32_t pin_setup_time; uint32_t pin_hold_time; uint32_t start_delay; };
struct pulsgen_msg_ch_t { uint32_t ch; };
struct pulsgen_msg_state_t { uint32_t state; };
struct pulsgen_msg_toggles_t { uint32_t toggles; };
struct pulsgen_msg_watchdog_setup_t { uint32_t enable; uint32_t time; };










// public methods
/**
 * @brief   setup GPIO pin for the selected channel
 *
 * @param   c           channel id
 * @param   port        GPIO port number
 * @param   pin         GPIO pin number
 * @param   inverted    invert pin state?
 *
 * @retval  none
 */
void pulsgen_pin_setup(uint8_t c, uint8_t port, uint8_t pin, uint8_t inverted)
{
    struct pulsgen_msg_pin_setup_t tx = *((struct pulsgen_msg_pin_setup_t *) &msg_buf);

    tx.ch = c;
    tx.port = port;
    tx.pin = pin;
    tx.inverted = inverted;

    msg_send(PULSGEN_MSG_PIN_SETUP, (uint8_t*)&tx, 4*4);
}

/**
 * @brief   setup a new task for the selected channel
 *
 * @param   c               channel id
 * @param   toggles         number of pin state changes
 * @param   pin_setup_time  pin state setup_time (in nanoseconds)
 * @param   pin_hold_time   pin state hold_time (in nanoseconds)
 * @param   start_delay     task start delay (in nanoseconds)
 *
 * @retval  none
 */
void pulsgen_task_setup
(
    uint32_t c,
    uint32_t toggles,
    uint32_t pin_setup_time,
    uint32_t pin_hold_time,
    uint32_t start_delay
)
{
    struct pulsgen_msg_task_setup_t tx = *((struct pulsgen_msg_task_setup_t *) &msg_buf);

    tx.ch = c;
    tx.toggles = toggles;
    tx.pin_setup_time = pin_setup_time;
    tx.pin_hold_time = pin_hold_time;
    tx.start_delay = start_delay;

    msg_send(PULSGEN_MSG_TASK_SETUP, (uint8_t*)&tx, 5*4);
}

/**
 * @brief   abort current task for the selected channel
 * @param   c       channel id
 * @retval  none
 */
void pulsgen_task_abort(uint8_t c)
{
    struct pulsgen_msg_ch_t tx = *((struct pulsgen_msg_ch_t *) &msg_buf);

    tx.ch = c;

    msg_send(PULSGEN_MSG_TASK_ABORT, (uint8_t*)&tx, 1*4);
}

/**
 * @brief   get current task state for the selected channel
 *
 * @param   c   channel id
 *
 * @retval  0   (channel have no task)
 * @retval  1   (channel have a task)
 */
uint8_t pulsgen_task_state(uint8_t c)
{
    static uint32_t n;
    struct pulsgen_msg_ch_t tx = *((struct pulsgen_msg_ch_t *) &msg_buf);
    struct pulsgen_msg_state_t rx = *((struct pulsgen_msg_state_t *) &msg_buf);

    tx.ch = c;

    msg_send(PULSGEN_MSG_TASK_STATE, (uint8_t*)&tx, 1*4);

    // finite loop, only 999999 tries to read an answer
    for ( n = 999999; n--; )
    {
        if ( msg_read(PULSGEN_MSG_TASK_STATE, (uint8_t*)&rx) < 0 ) continue;
        else return rx.state;
    }

    return 0;
}

/**
 * @brief   get current pin state changes since task start
 * @param   c   channel id
 * @retval  0..0xFFFFFFFF
 */
uint32_t pulsgen_task_toggles(uint8_t c)
{
    static uint32_t n;
    struct pulsgen_msg_ch_t tx = *((struct pulsgen_msg_ch_t *) &msg_buf);
    struct pulsgen_msg_toggles_t rx = *((struct pulsgen_msg_toggles_t *) &msg_buf);

    tx.ch = c;

    msg_send(PULSGEN_MSG_TASK_TOGGLES, (uint8_t*)&tx, 1*4);

    // finite loop, only 999999 tries to read an answer
    for ( n = 999999; n--; )
    {
        if ( msg_read(PULSGEN_MSG_TASK_TOGGLES, (uint8_t*)&rx) < 0 ) continue;
        else return rx.toggles;
    }

    return 0;
}

/**
 * @brief   enable/disable `abort all` watchdog
 * @param   enable      0 = disable watchdog, other values - enable watchdog
 * @param   time        watchdog wait time (in nanoseconds)
 * @retval  none
 */
void pulsgen_watchdog_setup(uint8_t enable, uint32_t time)
{
    struct pulsgen_msg_watchdog_setup_t tx =
        *((struct pulsgen_msg_watchdog_setup_t *) &msg_buf);

    tx.enable = enable;
    tx.time = time;

    msg_send(PULSGEN_MSG_WATCHDOG_SETUP, (uint8_t*)&tx, 2*4, 0);
}




#endif
