#include "rtapi.h"
#include "rtapi_app.h"
#include "hal.h"

MODULE_AUTHOR("test1");
MODULE_DESCRIPTION("test1");
MODULE_LICENSE("GPL");

static int32_t comp_id;
static const char * comp_name = "test1";
static hal_u32_t ** data;

static void data_update(void *arg, long period);




int32_t rtapi_app_main(void)
{
    // component init
    if ( (comp_id = hal_init(comp_name)) < 0 )
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: hal_init() failed\n", comp_name);
        return -1;
    }

    // memory allocation
    data = hal_malloc(sizeof(hal_u32_t));
    if ( !data )
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: hal_malloc() failed\n", comp_name);
        hal_exit(comp_id);
        return -1;
    }

    // export pins
    if ( 0 > hal_pin_u32_newf(HAL_IO, data, comp_id, "%s.data", comp_name) )
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: hal_pin_u32_newf() failed\n", comp_name);
        hal_exit(comp_id);
        return -1;
    }

    // export function
    if ( 0 > hal_export_functf(data_update, 0, 0, 0, comp_id, "%s.update", comp_name) )
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: %s.update function export failed\n", comp_name);
        hal_exit(comp_id);
        return -1;
    }

    return 0;
}




void rtapi_app_exit(void)
{
    hal_exit(comp_id);
}




static void data_update(void *arg, long period)
{
    (**data)++;
}
