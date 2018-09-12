/**
 * @file    arisc.c
 * @brief   Driver for the Allwinner ARISC CNC firmware
 * @note    ARISC CNC firmware: https://github.com/orangecnc/h3_arisc_firmware
 * @author  Mikhail Vydrenko (mikhail@vydrenko.ru)
 */

#include "rtapi.h"
#include "rtapi_app.h"
#include "hal.h"

#include "cpu.h"
#include "msg.h"
#include "gpio.h"
#include "stepgen_new.h"
#include "encoder.h"

#if !defined(TARGET_PLATFORM_ALLWINNER)
//#error "This driver is for the Allwinner platform only"
#endif

MODULE_AUTHOR("Mikhail Vydrenko");
MODULE_DESCRIPTION("Driver for the Allwinner ARISC CNC firmware");
MODULE_LICENSE("GPL");




static int32_t comp_id;
static const uint8_t * comp_name = "arisc";




int32_t rtapi_app_main(void)
{
    // get component id
    if ( (comp_id = hal_init(comp_name)) < 0 )
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: hal_init() failed\n", comp_name);
        return -1;
    }

    #define EXIT { hal_exit(comp_id); return -1; }

    // shared memory allocation and export
    cpu_id_get();
    if ( msg_mem_init(cpu_id, comp_name) ) EXIT;
    if ( gpio_malloc_and_export(comp_name, comp_id) ) EXIT;
    if ( stepgen_malloc_and_export(comp_name, comp_id) ) EXIT;

    // driver ready to work
    hal_ready(comp_id);

    return 0;
}




void rtapi_app_exit(void)
{
    msg_mem_deinit();
    hal_exit(comp_id);
}
