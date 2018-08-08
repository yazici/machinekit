/**
 * @file    encoder.h
 * @brief   ENCODER related data
 * @author  Mikhail Vydrenko (mikhail@vydrenko.ru)
 */

#ifndef _ENCODER_H
#define _ENCODER_H

#include <msg.h>




// public data

#define ENCODER_CH_CNT 8 ///< maximum number of encoder counter channels

enum { PHASE_A, PHASE_B, PHASE_Z };
enum { PH_A, PH_B, PH_Z };

/// messages types
enum
{
    ENCODER_MSG_PIN_SETUP = 0x30,
    ENCODER_MSG_SETUP,
    ENCODER_MSG_STATE_SET,
    ENCODER_MSG_STATE_GET,
    ENCODER_MSG_COUNTS_SET,
    ENCODER_MSG_COUNTS_GET
};

/// the message data access
struct encoder_msg_ch_t { uint32_t ch; };
struct encoder_msg_pin_setup_t { uint32_t ch; uint32_t phase; uint32_t port; uint32_t pin; };
struct encoder_msg_setup_t { uint32_t ch; uint32_t using_B; uint32_t using_Z; };
struct encoder_msg_state_set_t { uint32_t ch; uint32_t state; };
struct encoder_msg_counts_set_t { uint32_t ch; int32_t counts; };
struct encoder_msg_state_get_t { uint32_t state; };
struct encoder_msg_counts_get_t { int32_t counts; };








// public methods

/**
 * @brief   setup encoder pin for the selected channel and phase
 *
 * @param   c           channel id
 * @param   phase       PHASE_A..PHASE_Z
 * @param   port        GPIO port number
 * @param   pin         GPIO pin number
 *
 * @retval  none
 */
void encoder_pin_setup(uint8_t c, uint8_t phase, uint8_t port, uint8_t pin)
{
    struct encoder_msg_pin_setup_t tx = *((struct encoder_msg_pin_setup_t *) &msg_buf);

    tx.ch = c;
    tx.phase = phase;
    tx.port = port;
    tx.pin = pin;

    msg_send(ENCODER_MSG_PIN_SETUP, (uint8_t*)&tx, 4*4, 0);
}

/**
 * @brief   setup selected channel of encoder counter
 *
 * @param   c           channel id
 * @param   using_B     use phase B input?
 * @param   using_Z     use phase Z index input?
 *
 * @retval  none
 */
void encoder_setup(uint8_t c, uint8_t using_B, uint8_t using_Z)
{
    struct encoder_msg_setup_t tx = *((struct encoder_msg_setup_t *) &msg_buf);

    tx.ch = c;
    tx.using_B = using_B;
    tx.using_Z = using_Z;

    msg_send(ENCODER_MSG_SETUP, (uint8_t*)&tx, 3*4, 0);
}

/**
 * @brief   enable/disable selected channel of encoder counter
 * @param   c       channel id
 * @retval  none
 */
void encoder_state_set(uint8_t c, uint8_t state)
{
    struct encoder_msg_state_set_t tx = *((struct encoder_msg_state_set_t *) &msg_buf);

    tx.ch = c;
    tx.state = state;

    msg_send(ENCODER_MSG_STATE_SET, (uint8_t*)&tx, 2*4, 0);
}

/**
 * @brief   change number of counts for the selected channel
 * @param   c       channel id
 * @param   counts  new value for encoder channel counts
 * @retval  none
 */
void encoder_counts_set(uint8_t c, int32_t counts)
{
    struct encoder_msg_counts_set_t tx = *((struct encoder_msg_counts_set_t *) &msg_buf);

    tx.ch = c;
    tx.counts = counts;

    msg_send(ENCODER_MSG_COUNTS_SET, (uint8_t*)&tx, 2*4, 0);
}

/**
 * @brief   get state for the selected channel
 *
 * @param   c   channel id
 *
 * @retval  0   (channel is disabled)
 * @retval  1   (channel is enabled)
 */
uint8_t encoder_state_get(uint8_t c)
{
    struct encoder_msg_ch_t tx = *((struct encoder_msg_ch_t *) &msg_buf);
    struct encoder_msg_state_get_t rx = *((struct encoder_msg_state_get_t *) &msg_buf);

    tx.ch = c;

    msg_send(ENCODER_MSG_STATE_GET, (uint8_t*)&tx, 1*4, 0);

    // finite loop, only 999999 tries to read an answer
    uint32_t n = 0;
    for ( n = 999999; n--; )
    {
        if ( msg_read(ENCODER_MSG_STATE_GET, (uint8_t*)&rx) < 0 ) continue;
        else return rx.state;
    }

    return 0;
}

/**
 * @brief   get current counts for the selected channel
 * @param   c   channel id
 * @retval  signed 32-bit number
 */
int32_t encoder_counts_get(uint8_t c)
{
    struct encoder_msg_ch_t tx = *((struct encoder_msg_ch_t *) &msg_buf);
    struct encoder_msg_counts_get_t rx = *((struct encoder_msg_counts_get_t *) &msg_buf);

    tx.ch = c;

    msg_send(ENCODER_MSG_COUNTS_GET, (uint8_t*)&tx, 1*4, 0);

    // finite loop, only 999999 tries to read an answer
    uint32_t n = 0;
    for ( n = 999999; n--; )
    {
        if ( msg_read(ENCODER_MSG_COUNTS_GET, (uint8_t*)&rx) < 0 ) continue;
        else return rx.counts;
    }

    return 0;
}




#endif
