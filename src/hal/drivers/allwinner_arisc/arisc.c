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




// private vars

static int32_t comp_id;
static const uint8_t * comp_name = "arisc";
static int32_t mem_state = 0;




// main entry

int32_t rtapi_app_main(void)
{
    // component init
    comp_id = hal_init(comp_name);
    if (comp_id < 0)
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




    // arisc shared memory init
    mem_state = mem_init(H3);
    switch (mem_state)
    {
        case -1: { PRINT_AND_EXIT("can't open /dev/mem file"); break; }
        case -2: { PRINT_AND_EXIT("mmap() failed"); break; }
    }

    return 0;
}

void rtapi_app_exit(void)
{
    if ( !mem_state ) mem_deinit();
    hal_exit(comp_id);
}
