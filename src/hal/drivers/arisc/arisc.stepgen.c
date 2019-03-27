#include "rtapi.h"
#include "rtapi_app.h"
#include "hal.h"
#include <float.h>
#include <stdlib.h>
#include "rtapi_math.h"

#include "allwinner_CPU.h"
#include "msg_api.h"
#include "gpio_api.h"
#include "stepgen_api.h"
#include "arisc.stepgen.h"




#if !defined(TARGET_PLATFORM_ALLWINNER)
//#error "This driver is for the Allwinner platform only"
#endif

MODULE_DESCRIPTION("stepgen driver for the Allwinner ARISC firmware");
MODULE_LICENSE("GPL");




#define g *sg[ch]
#define gp sg[ch]

#define EXPORT_PIN(IO_TYPE,VAR_TYPE,VAL,NAME,DEFAULT) \
    r += hal_pin_##VAR_TYPE##_newf(IO_TYPE, &(gp.VAL), comp_id,\
    "%s.%d." NAME, comp_name, ch);\
    g.VAL = DEFAULT;

#define EXPORT_FUNC(FUNC,NAME,FP) \
    hal_export_functf(FUNC, 0, FP, 0, comp_id, "%s."NAME, comp_name)

#define PRINT_ERROR(MSG) \
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: "MSG"\n", comp_name)

#define PRINT_ERROR_AND_RETURN(MSG,RETVAL) \
    { PRINT_ERROR(MSG); return RETVAL; }




static int32_t comp_id;
static const uint8_t * comp_name = "arisc.stepgen";
static uint8_t cpu_id = ALLWINNER_H3;

static char *CPU;
RTAPI_MP_STRING(CPU, "Allwinner CPU name");

static char *ctrl_type;
RTAPI_MP_STRING(ctrl_type, "channels control type, comma separated");

static stepgen_ch_t *sg;
static uint8_t sg_cnt = 0;
static uint8_t wd = 0;




// TOOLS

static hal_bit_t floats_equal(hal_float_t f1, hal_float_t f2)
{
    return *((int64_t*) &f1) == *((int64_t*) &f2);
}

static void update_accel_max(uint8_t ch)
{
    if ( floats_equal(g.accel_max, gp.accel_max_old) ) return;
    if ( g.accel_max < 0 ) g.accel_max = 1.0;

    gp.step_accel_max   = (hal_u32_t) (g.accel_max * g.pos_scale);
    gp.accel_max_old    = g.accel_max;
}

static void update_pos_scale(uint8_t ch)
{
    if ( g.pos_scale < 0 || (g.pos_scale < 1e-20 && g.pos_scale > -1e-20) ) g.pos_scale = 1.0;
}

static void update_vel_max(uint8_t ch)
{
    if ( !floats_equal(g.vel_max, gp.vel_max_old) )
    {
        if ( g.vel_max < 0 ) g.vel_max = 1.0;
        gp.step_freq_max    = (hal_u32_t) (g.vel_max * g.pos_scale);
        gp.vel_max_old      = g.vel_max;
    }

    if ( g.step_len != gp.step_len_old || g.step_space != gp.step_space_old )
    {
        if ( !g.step_len && !g.step_space ) g.step_len = 1;

        hal_u32_t freq_max = 1000000000 / (g.step_len + g.step_space);
        if ( gp.step_freq_max > freq_max ) gp.step_freq_max = freq_max;

        gp.step_len_old    = g.step_len;
        gp.step_space_old  = g.step_space;
    }
}

static void update_vel_cmd(uint8_t ch)
{
    if ( floats_equal(g.vel_cmd, gp.vel_cmd_old) ) return;

    gp.step_freq_new = (hal_s32_t) (g.vel_cmd * g.pos_scale);
    gp.vel_cmd_old = g.vel_cmd;
}

static void update_pos_cmd(uint8_t ch)
{
    if ( floats_equal(g.pos_cmd, gp.pos_cmd_old) ) return;

    gp.counts_new = (hal_s32_t) (g.pos_cmd * g.pos_scale);
    gp.pos_cmd_old = g.pos_cmd;
}

static void update_pins(uint8_t ch)
{
    if ( g.step_port != gp.step_port_old    ||
         g.step_pin  != gp.step_pin_old     ||
         g.step_inv  != gp.step_inv_old )
    {
        if ( g.step_port < GPIO_PORTS_CNT && g.step_pin < GPIO_PINS_CNT )
        {
            stepgen_pin_setup(ch, 0, g.step_port, g.step_pin, g.step_inv);
        }

        gp.step_port_old = g.step_port;
        gp.step_pin_old  = g.step_pin;
        gp.step_inv_old  = g.step_inv;
    }

    if ( g.dir_port != gp.dir_port_old  ||
         g.dir_pin  != gp.dir_pin_old   ||
         g.dir_inv  != gp.dir_inv_old )
    {
        if ( g.dir_port < GPIO_PORTS_CNT && g.dir_pin < GPIO_PINS_CNT )
        {
            stepgen_pin_setup(ch, 1, g.dir_port, g.dir_pin, g.dir_inv);
        }

        gp.dir_port_old = g.dir_port;
        gp.dir_pin_old  = g.dir_pin;
        gp.dir_inv_old  = g.dir_inv;
    }
}

static void update_stepdir_time(uint8_t ch)
{
    if ( !g.step_len )   { g.step_len   = 5000;  gp.step_len_old   = g.step_len; }
    if ( !g.step_space ) { g.step_space = 5000;  gp.step_space_old = g.step_space; }

    if ( !g.dir_setup ) g.dir_setup = 35000;
    if ( !g.dir_hold )  g.dir_hold  = 35000;
}




// HAL functions

static void capture_pos(void *arg, long period)
{
    static uint8_t ch;

    for ( ch = sg_cnt; ch--; )
    {
        if ( !g.enable ) continue;

        g.counts = stepgen_pos_get(ch);
        g.rawcounts = g.counts;

        update_pos_scale(ch);
        g.pos_fb = ((hal_float_t)g.counts) / g.pos_scale;
    }
}

static void update_freq(void *arg, long period)
{
    static uint8_t ch;

    // is watchdog disabled?
    if ( !wd )
    {
        // enable watchdog
        wd = 1;
        // start watchdod, wait time = 5 periods
        stepgen_watchdog_setup(1, 5*period);
    }

    for ( ch = sg_cnt; ch--; )
    {
        if ( !g.enable )
        {
            if ( gp.step_freq )
            {
                gp.step_freq = 0;
                g.freq = ((hal_float_t)gp.step_freq) / g.pos_scale;
            }
            continue;
        }

        // recalculate private and public data
        update_stepdir_time(ch);
        update_pins(ch);
        update_pos_scale(ch);
        update_accel_max(ch);
        update_vel_max(ch);

        if ( gp.ctrl_type ) // velocity mode
        {
            uint32_t period_accel_max, diff, low_time, high_time;

            update_vel_cmd(ch);

            if ( gp.step_freq != gp.step_freq_new )
            {
                period_accel_max = (uint32_t)(((uint64_t)gp.step_accel_max) * period / 1000000000);

                diff = abs(gp.step_freq - gp.step_freq_new);
                if ( diff > period_accel_max ) diff = period_accel_max;

                gp.step_freq += gp.step_freq < gp.step_freq_new ? diff : -diff;
            }

            g.freq = ((hal_float_t)gp.step_freq) / g.pos_scale;

            if ( (1000000000 / gp.step_freq) > g.step_len )
            {
                low_time = (1000000000 / gp.step_freq) - g.step_len;
                high_time = g.step_len;
            }
            else
            {
                low_time = g.step_len / 2;
                high_time = low_time;
            }

#warning "ATTENTION: arisc firmware can abort next task too"
            stepgen_abort(ch);
            stepgen_task_add(ch, 0, UINT32_MAX, low_time, high_time);
        }
        else // position mode
        {
            update_pos_cmd(ch);
            if ( g.counts == gp.counts_new ) continue;
        }
    }
}




// INIT

static int32_t malloc_and_export(const char *comp_name, int32_t comp_id)
{
    int32_t r, ch;
    int8_t *data = ctrl_type, *token, type[STEPGEN_CH_CNT_MAX] = {0};

    // get channels count and type

    while ( (token = strtok(data, ",")) != NULL )
    {
        if ( data != NULL ) data = NULL;

        if      ( token[0] == 'P' || token[0] == 'p' ) type[sg_cnt++] = 0;
        else if ( token[0] == 'V' || token[0] == 'v' ) type[sg_cnt++] = 1;
    }

    if ( !sg_cnt ) return 0;
    if ( sg_cnt > STEPGEN_CH_CNT_MAX ) sg_cnt = STEPGEN_CH_CNT_MAX;

    // shared memory allocation

    sg = hal_malloc(sg_cnt * sizeof(stepgen_ch_t));
    if ( !sg ) PRINT_ERROR_AND_RETURN("hal_malloc() failed", -1);

    // export HAL pins and set default values

    for ( r = 0, ch = sg_cnt; ch--; )
    {
        EXPORT_PIN(HAL_IN,bit,enable,"enable", 0);
        EXPORT_PIN(HAL_IN,u32,step_space,"stepspace", 1000);
        EXPORT_PIN(HAL_IN,u32,step_len,"steplen", 1000);
        EXPORT_PIN(HAL_IN,u32,dir_setup,"dirsetup", 1000);
        EXPORT_PIN(HAL_IN,u32,dir_hold,"dirhold", 1000);
        EXPORT_PIN(HAL_IN,u32,step_port,"step-port", UINT32_MAX);
        EXPORT_PIN(HAL_IN,u32,step_pin,"step-pin", UINT32_MAX);
        EXPORT_PIN(HAL_IN,bit,step_inv,"step-invert", 0);
        EXPORT_PIN(HAL_IN,u32,dir_port,"dir-port", UINT32_MAX);
        EXPORT_PIN(HAL_IN,u32,dir_pin,"dir-pin", UINT32_MAX);
        EXPORT_PIN(HAL_IN,bit,dir_inv,"dir-invert", 0);
        EXPORT_PIN(HAL_IN,float,pos_scale,"position-scale", 1.0);
        EXPORT_PIN(HAL_IN,float,vel_max,"maxvel", 0.0);
        EXPORT_PIN(HAL_IN,float,accel_max,"maxaccel", 0.0);
        EXPORT_PIN(HAL_OUT,float,pos_fb,"position-fb", 0.0);
        EXPORT_PIN(HAL_OUT,float,freq,"frequency", 0.0);
        EXPORT_PIN(HAL_OUT,s32,counts,"counts", 0);
        EXPORT_PIN(HAL_OUT,s32,rawcounts,"rawcounts", 0);

        if ( type[ch] ) { EXPORT_PIN(HAL_IN,float,vel_cmd,"velocity-cmd", 0.0); }
        else            { EXPORT_PIN(HAL_IN,float,pos_cmd,"position-cmd", 0.0); }

        gp.counts_new = 0;
        gp.ctrl_type = type[ch];

        gp.step_inv_old = 0;
        gp.step_pin_old = UINT32_MAX;
        gp.step_port_old = UINT32_MAX;
        gp.step_len_old = 1000;
        gp.step_space_old = 1000;
        gp.step_freq = 0;
        gp.step_freq_new = 0;
        gp.step_freq_max = 0;
        gp.step_accel_max = 0;

        gp.dir_inv_old = 0;
        gp.dir_pin_old = UINT32_MAX;
        gp.dir_port_old = UINT32_MAX;

        gp.vel_max_old = 0.0;
        gp.accel_max_old = 0.0;
    }
    if ( r ) PRINT_ERROR_AND_RETURN("HAL pins export failed", -1);

    // export HAL functions

    r = 0;
    r += EXPORT_FUNC(capture_pos, "capture-position", 1);
    r += EXPORT_FUNC(update_freq, "update-freq", 1);
    if ( r ) PRINT_ERROR_AND_RETURN("HAL functions export failed", -1);

    return 0;
}




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
    cpu_id = allwinner_cpu_id_get(CPU);
    if ( msg_mem_init(cpu_id, comp_name) ) EXIT;
    if ( malloc_and_export(comp_name, comp_id) ) EXIT;

    // driver ready to work
    hal_ready(comp_id);

    return 0;
}

void rtapi_app_exit(void)
{
    msg_mem_deinit();
    hal_exit(comp_id);
}




#undef EXPORT_PIN
#undef PRINT_ERROR
#undef PRINT_ERROR_AND_RETURN
#undef g
#undef gp
