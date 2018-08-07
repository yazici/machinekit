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

#include "cpu.h"
#include "msg.h"
#include "gpio.h"
#include "pulsgen.h"
#include "encoder.h"

#if !defined(TARGET_PLATFORM_ALLWINNER)
//#error "This driver is for the Allwinner platform only"
#endif

MODULE_AUTHOR("Mikhail Vydrenko");
MODULE_DESCRIPTION("Driver for the Allwinner ARISC CNC firmware");
MODULE_LICENSE("GPL");








static int32_t comp_id;
static const uint8_t * comp_name = "arisc";

static int8_t *cpu;
RTAPI_MP_STRING(cpu, "CPU name");
static int8_t *gpio_in;
RTAPI_MP_STRING(gpio_in, "GPIO input pins, comma separated");
static int8_t *gpio_out;
RTAPI_MP_STRING(gpio_out, "GPIO output pins, comma separated");








int32_t rtapi_app_main(void)
{
    // component init
    if ( (comp_id = hal_init(comp_name)) < 0 )
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: hal_init() failed\n", comp_name);
        return -1;
    }

    // get CPU id from arguments
    cpu_id_get(cpu);

#define EXIT { hal_exit(comp_id); return -1; }

    // arisc shared memory init
    if ( msg_mem_init(cpu_id, comp_name) ) EXIT;

    // HAL shared memory allocation
    if ( gpio_pins_malloc(comp_name) ) EXIT;

    // configure and export GPIO pins
    if ( gpio_pins_export(gpio_in, GPIO_INPUT_PINS, comp_name, comp_id) ) EXIT;
    if ( gpio_pins_export(gpio_out, GPIO_OUTPUT_PINS, comp_name, comp_id) ) EXIT;

    // export GPIO functions
    if ( gpio_func_export(comp_name, comp_id) ) EXIT;

#undef EXIT

    hal_ready(comp_id);

    return 0;
}




void rtapi_app_exit(void)
{
    msg_mem_deinit();
    hal_exit(comp_id);
}
