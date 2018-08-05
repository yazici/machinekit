/**
 * @file    gpio.h
 * @brief   GPIO related data
 * @author  Mikhail Vydrenko (mikhail@vydrenko.ru)
 */

#ifndef _GPIO_H
#define _GPIO_H

#include <stdint.h>
#include <string.h>
#include "hal_types.h"
#include "msg.h"




#define GPIO_PORTS_CNT  8   ///< number of GPIO ports
#define GPIO_PINS_CNT   32 ///< number of GPIO port pins

/// the GPIO port names
enum { PA, PB, PC, PD, PE, PF, PG, PL };

/// the GPIO pin states
enum { LOW, HIGH };

/// the message types
enum
{
    GPIO_MSG_SETUP_FOR_OUTPUT = 0x10,
    GPIO_MSG_SETUP_FOR_INPUT,

    GPIO_MSG_PIN_GET,
    GPIO_MSG_PIN_SET,
    GPIO_MSG_PIN_CLEAR,

    GPIO_MSG_PORT_GET,
    GPIO_MSG_PORT_SET,
    GPIO_MSG_PORT_CLEAR
};

enum { GPIO_INPUT_PINS, GPIO_OUTPUT_PINS };

/// the message data access
struct gpio_msg_port_t      { uint32_t port; };
struct gpio_msg_port_pin_t  { uint32_t port; uint32_t pin;  };
struct gpio_msg_port_mask_t { uint32_t port; uint32_t mask; };
struct gpio_msg_state_t     { uint32_t state; };








static const char *gpio_port_name[GPIO_PORTS_CNT] =
    {"PA","PB","PC","PD","PE","PF","PG","PL"};

hal_bit_t **gpio_pin_state[GPIO_PORTS_CNT][GPIO_PINS_CNT] = {{0}};
hal_bit_t **gpio_pin_state_inv[GPIO_PORTS_CNT][GPIO_PINS_CNT] = {{0}};

static uint32_t gpio_port_state[GPIO_PORTS_CNT] = {0};
static uint32_t gpio_port_state_prev[GPIO_PORTS_CNT] = {0};

static uint32_t gpio_port_output_mask[GPIO_PORTS_CNT] = {0};
static uint32_t gpio_port_input_mask[GPIO_PORTS_CNT] = {0};









/**
 * @brief   set pin mode to OUTPUT
 * @param   port    GPIO port number    (0 .. GPIO_PORTS_CNT)
 * @param   pin     GPIO pin number     (0 .. GPIO_PINS_CNT)
 * @retval  none
 */
void gpio_pin_setup_for_output(uint32_t port, uint32_t pin)
{
    struct gpio_msg_port_pin_t tx = *((struct gpio_msg_port_pin_t *) &msg_buf);

    tx.port = port;
    tx.pin  = pin;

    msg_send(GPIO_MSG_SETUP_FOR_OUTPUT, (uint8_t*)&tx, 2*4, 0);
}

/**
 * @brief   set pin mode to INPUT
 * @param   port    GPIO port number    (0 .. GPIO_PORTS_CNT)
 * @param   pin     GPIO pin number     (0 .. GPIO_PINS_CNT)
 * @retval  none
 */
void gpio_pin_setup_for_input(uint32_t port, uint32_t pin)
{
    struct gpio_msg_port_pin_t tx = *((struct gpio_msg_port_pin_t *) &msg_buf);

    tx.port = port;
    tx.pin  = pin;

    msg_send(GPIO_MSG_SETUP_FOR_INPUT, (uint8_t*)&tx, 2*4, 0);
}

/**
 * @brief   get pin state
 * @param   port    GPIO port number    (0 .. GPIO_PORTS_CNT)
 * @param   pin     GPIO pin number     (0 .. GPIO_PINS_CNT)
 * @retval  1 (HIGH)
 * @retval  0 (LOW)
 */
uint32_t gpio_pin_get(uint32_t port, uint32_t pin)
{
    struct gpio_msg_port_pin_t tx = *((struct gpio_msg_port_pin_t *) &msg_buf);
    struct gpio_msg_state_t rx = *((struct gpio_msg_state_t *) &msg_buf);

    tx.port = port;
    tx.pin = pin;

    msg_send(GPIO_MSG_PIN_GET, (uint8_t*)&tx, 2*4, 0);

    // finite loop, only 999999 tries to read an answer
    uint32_t n = 0;
    for ( n = 999999; n--; )
    {
        if ( msg_read(GPIO_MSG_PIN_GET, (uint8_t*)&rx, 0) < 0 ) continue;
        else return rx.state;
    }

    return 0;
}

/**
 * @brief   set pin state to HIGH (1)
 * @param   port    GPIO port number    (0 .. GPIO_PORTS_CNT)
 * @param   pin     GPIO pin number     (0 .. GPIO_PINS_CNT)
 * @retval  none
 */
void gpio_pin_set(uint32_t port, uint32_t pin)
{
    struct gpio_msg_port_pin_t tx = *((struct gpio_msg_port_pin_t *) &msg_buf);

    tx.port = port;
    tx.pin  = pin;

    msg_send(GPIO_MSG_PIN_SET, (uint8_t*)&tx, 2*4, 0);
}

/**
 * @brief   set pin state to LOW (0)
 * @param   port    GPIO port number    (0 .. GPIO_PORTS_CNT)
 * @param   pin     GPIO pin number     (0 .. GPIO_PINS_CNT)
 * @retval  none
 */
void gpio_pin_clear(uint32_t port, uint32_t pin)
{
    struct gpio_msg_port_pin_t tx = *((struct gpio_msg_port_pin_t *) &msg_buf);

    tx.port = port;
    tx.pin  = pin;

    msg_send(GPIO_MSG_PIN_CLEAR, (uint8_t*)&tx, 2*4, 0);
}

/**
 * @brief   get port state
 * @param   port    GPIO port number (0 .. GPIO_PORTS_CNT)
 * @note    each bit value of returned value represents port pin state
 * @retval  0 .. 0xFFFFFFFF
 */
uint32_t gpio_port_get(uint32_t port)
{
    struct gpio_msg_port_t tx = *((struct gpio_msg_port_t *) &msg_buf);
    struct gpio_msg_state_t rx = *((struct gpio_msg_state_t *) &msg_buf);

    tx.port = port;

    msg_send(GPIO_MSG_PORT_GET, (uint8_t*)&tx, 1*4, 0);

    // finite loop, only 999999 tries to read an answer
    uint32_t n = 0;
    for ( n = 999999; n--; )
    {
        if ( msg_read(GPIO_MSG_PORT_GET, (uint8_t*)&rx, 0) < 0 ) continue;
        else return rx.state;
    }

    return 0;
}

/**
 * @brief   set port pins state by mask
 *
 * @param   port    GPIO port number        (0 .. GPIO_PORTS_CNT)
 * @param   mask    GPIO pins mask to set   (0 .. 0xFFFFFFFF) \n\n
 *                  mask examples: \n\n
 *                      mask = 0xFFFFFFFF (0b11111111111111111111111111111111) means <b>set all pins state to 1 (HIGH)</b> \n
 *                      mask = 0x00000001 (0b1) means <b>set pin 0 state to 1 (HIGH)</b> \n
 *                      mask = 0x0000000F (0b1111) means <b>set pins 0,1,2,3 states to 1 (HIGH)</b>
 *
 * @retval  none
 */
void gpio_port_set(uint32_t port, uint32_t mask)
{
    struct gpio_msg_port_mask_t tx = *((struct gpio_msg_port_mask_t *) &msg_buf);

    tx.port = port;
    tx.mask = mask;

    msg_send(GPIO_MSG_PORT_SET, (uint8_t*)&tx, 2*4, 0);
}

/**
 * @brief   clear port pins state by mask
 *
 * @param   port    GPIO port number        (0 .. GPIO_PORTS_CNT)
 * @param   mask    GPIO pins mask to clear (0 .. 0xFFFFFFFF) \n\n
 *                  mask examples: \n\n
 *                  mask = 0xFFFFFFFF (0b11111111111111111111111111111111) means <b>set all pins state to 0 (LOW)</b> \n
 *                  mask = 0x00000003 (0b11) means <b>set pins 0,1 states to 0 (LOW)</b> \n
 *                  mask = 0x00000008 (0b1000) means <b>set pin 3 state to 0 (LOW)</b>
 *
 * @retval  none
 */
void gpio_port_clear(uint32_t port, uint32_t mask)
{
    struct gpio_msg_port_mask_t tx = *((struct gpio_msg_port_mask_t *) &msg_buf);

    tx.port = port;
    tx.mask = mask;

    msg_send(GPIO_MSG_PORT_CLEAR, (uint8_t*)&tx, 2*4, 0);
}








static int32_t gpio_pins_export
(
    char *arg_str,
    uint8_t type,
    const char *comp_name,
    int32_t comp_id
)
{
    if ( arg_str == NULL ) return 0;

    int8_t *data = arg_str, *token;
    uint8_t port, pin, found;
    int32_t retval;

    // break input_pins string by comma
    while ( (token = strtok(data, ",")) != NULL )
    {
        if ( data != NULL ) data = NULL;
        if ( strlen(token) < 3 ) continue;

        // trying to find a correct port name
        for ( found = 0, port = GPIO_PORTS_CNT; port--; )
        {
            if ( 0 == memcmp(token, gpio_port_name[port], 2) )
            {
                found = 1;
                break;
            }
        }

        if ( !found ) continue;

        // trying to find a correct pin number
        pin = (uint8_t) strtoul(&token[2], NULL, 10);

        if ( (pin == 0 && token[2] != '0') || pin >= GPIO_PINS_CNT ) continue;

        // export pin function
        retval = hal_pin_bit_newf( (type == GPIO_INPUT_PINS ? HAL_OUT : HAL_IN),
            gpio_pin_state[port][pin], comp_id, "%s.gpio.%s-in",
            comp_name, token, (type == GPIO_INPUT_PINS ? "in" : "out") );

        // export pin inverted function
        retval = hal_pin_bit_newf( (type == GPIO_INPUT_PINS ? HAL_OUT : HAL_IN),
            gpio_pin_state[port][pin], comp_id, "%s.gpio.%s-not",
            comp_name, token, (type == GPIO_INPUT_PINS ? "in" : "out") );

        if (retval < 0)
        {
            rtapi_print_msg(RTAPI_MSG_ERR, "%s: [GPIO] pin %s export failed \n",
                comp_name, token);
            return -1;
        }

        // configure GPIO pin
        if ( type == GPIO_INPUT_PINS )
        {
            gpio_port_input_mask[port] |= 1U << pin;
            gpio_pin_setup_for_input(port, pin);
        }
        else
        {
            gpio_port_output_mask[port] |= 1U << pin;
            gpio_pin_setup_for_output(port, pin);
        }
    }

    return 0;
}




static int32_t gpio_pins_malloc
(
    const char *comp_name
)
{
    uint8_t port, pin;
    for ( port = GPIO_PORTS_CNT; port--; )
    {
        for ( pin = GPIO_PINS_CNT; pin--; )
        {
            gpio_pin_state[port][pin] = hal_malloc(sizeof(hal_bit_t *));
            gpio_pin_state_inv[port][pin] = hal_malloc(sizeof(hal_bit_t *));

            if ( !gpio_pin_state[port][pin] || !gpio_pin_state_inv[port][pin] )
            {
                rtapi_print_msg(RTAPI_MSG_ERR,
                    "%s: [GPIO] pin %s%d hal_malloc() failed \n",
                    comp_name, gpio_port_name[port], pin);
                return -1;
            }
        }
    }

    return 0;
}




#endif
