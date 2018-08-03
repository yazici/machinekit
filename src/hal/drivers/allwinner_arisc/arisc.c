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

#if !defined(TARGET_PLATFORM_ALLWINNER)
//#error "This driver is for the Allwinner platform only"
#endif

#include "h3.h"

MODULE_AUTHOR("Mikhail Vydrenko");
MODULE_DESCRIPTION("Driver for the Allwinner ARISC CNC firmware");
MODULE_LICENSE("GPL");




static int32_t comp_id;
static const uint8_t * comp_name = "arisc";




int32_t rtapi_app_main(void)
{
    mem_init();

    comp_id = hal_init(comp_name);
    if (comp_id < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: hal_init() failed\n", comp_name);
        return -1;
    }

    return 0;
}

void rtapi_app_exit(void)
{
    mem_deinit();
    hal_exit(comp_id);
}
