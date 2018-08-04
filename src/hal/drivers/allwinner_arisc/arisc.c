/**
 * @file    arisc.c
 * @brief   Driver for the Allwinner ARISC CNC firmware
 * @note    ARISC CNC firmware: https://github.com/orangecnc/h3_arisc_firmware
 * @author  Mikhail Vydrenko (mikhail@vydrenko.ru)
 */

#include "rtapi.h"          /* RTAPI realtime OS API */
#include "rtapi_app.h"      /* RTAPI realtime module decls */
                            /* this also includes config.h */
#include "hal.h"            /* HAL public API decls */
#include "api.h"

#if !defined(TARGET_PLATFORM_ALLWINNER)
//#error "This driver is for the Allwinner platform only"
#endif

MODULE_AUTHOR("Mikhail Vydrenko");
MODULE_DESCRIPTION("Driver for the Allwinner ARISC CNC firmware");
MODULE_LICENSE("GPL");








static int32_t comp_id;
static const uint8_t * comp_name = "arisc";

static int32_t mem_state = 0;

static int8_t *CPU;
RTAPI_MP_STRING(CPU, "CPU name");
static int8_t *in;
RTAPI_MP_STRING(in, "input pins, comma separated");
static int8_t *out;
RTAPI_MP_STRING(out, "output pins, comma separated");

hal_bit_t **gpio_pin_state[GPIO_PORTS_CNT][GPIO_PINS_CNT] = {{0}};
hal_bit_t **gpio_pin_state_inv[GPIO_PORTS_CNT][GPIO_PINS_CNT] = {{0}};

static uint32_t gpio_port_state[GPIO_PORTS_CNT] = {0};
static uint32_t gpio_port_state_prev[GPIO_PORTS_CNT] = {0};

static uint32_t gpio_port_output_mask[GPIO_PORTS_CNT] = {0};
static uint32_t gpio_port_input_mask[GPIO_PORTS_CNT] = {0};








int32_t rtapi_app_main(void)
{
    // component init
    comp_id = hal_init(comp_name);
    if ( comp_id < 0 )
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: hal_init() failed\n", comp_name);
        return -1;
    }

#define PRINT_AND_EXIT(MSG) \
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: " MSG "\n", comp_name);\
    hal_exit(comp_id);\
    return -1;

#define PRINT_AND_EXIT_P1(MSG,P1) \
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: " MSG "\n", comp_name, P1);\
    hal_exit(comp_id);\
    return -1;

#define PRINT_AND_EXIT_P2(MSG,P1,P2) \
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: " MSG "\n", comp_name, P1, P2);\
    hal_exit(comp_id);\
    return -1;

    // get CPU id from arguments
    if ( CPU != NULL )
    {
        uint8_t c;
        for ( c = CPU_CNT; c--; )
        {
            if ( 0 == strcmp(CPU, cpu_data[c].name) )
            {
                cpu_id = c;
                break;
            }
        }
    }

    // arisc shared memory init
    mem_state = mem_init(cpu_id);
    switch (mem_state)
    {
        case -1: { PRINT_AND_EXIT("can't open /dev/mem file"); break; }
        case -2: { PRINT_AND_EXIT("mmap() failed"); break; }
    }

    // HAL shared memory allocation
    {
        uint8_t port, pin;
        for ( port = GPIO_PORTS_CNT; port--; )
        {
            for ( port = GPIO_PINS_CNT; pin--; )
            {
                gpio_pin_state[port][pin] = hal_malloc(sizeof(hal_bit_t *));
                gpio_pin_state_inv[port][pin] = hal_malloc(sizeof(hal_bit_t *));
                if ( !gpio_pin_state[port][pin] || !gpio_pin_state_inv[port][pin] )
                {
                    PRINT_AND_EXIT("hal_malloc() failed");
                }
            }
        }
    }

    // configure and export GPIO input pins
    if ( in != NULL )
    {
        int8_t *data = in, *token;
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
                if ( 0 == strcmp(token, gpio_port_name[port]) )
                {
                    found = 1;
                    break;
                }
            }

            if ( !found ) continue;

            // trying to find a correct pin number
            pin = (uint8_t) strtoul(&token[2], NULL, 10);

            if ( (pin == 0 && token[2] != '0') || pin >= GPIO_PINS_CNT ) continue;

            // export pin input function
            retval = hal_pin_bit_newf (
                HAL_OUT, &gpio_pin_state[port][pin], comp_id,
                "%s.gpio.%s-in", comp_name, token );

            // export pin inverted input function
            retval += hal_pin_bit_newf (
                HAL_OUT, &gpio_pin_state_inv[port][pin], comp_id,
                "%s.gpio.%s-in-not", comp_name, token );

            if (retval < 0) { PRINT_AND_EXIT_P1("GPIO pin %s export failed", token); }

            // configure GPIO pin for input
            gpio_port_input_mask[port] |= 1U << pin;
            gpio_pin_setup_for_input(port, pin);
        }
    }

    return 0;
}

void rtapi_app_exit(void)
{
    if ( !mem_state ) mem_deinit();
    hal_exit(comp_id);
}
