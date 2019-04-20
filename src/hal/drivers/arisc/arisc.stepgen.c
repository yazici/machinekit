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

#define E (1000000UL) // epsilon, using to calculate double as long long int




static int32_t comp_id;
static const uint8_t * comp_name = "arisc.stepgen";
static uint8_t cpu_id = ALLWINNER_H3;

static char *CPU = "H3";
RTAPI_MP_STRING(CPU, "Allwinner CPU name");

static char *ctrl_type = "";
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

    gp.step_accel_max   = E * (hal_u64_t)(g.accel_max * g.pos_scale);
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
        gp.step_freq_max    = E * (hal_u64_t)(g.vel_max * g.pos_scale);
        gp.vel_max_old      = g.vel_max;
    }

    if ( g.step_len != gp.step_len_old || g.step_space != gp.step_space_old )
    {
        if ( !g.step_len && !g.step_space ) g.step_len = 1;

        hal_u64_t freq_max = (uint64_t)(E * 1000000000) / (g.step_len + g.step_space);
        if ( gp.step_freq_max > freq_max ) gp.step_freq_max = freq_max;

        gp.step_len_old    = g.step_len;
        gp.step_space_old  = g.step_space;
    }
}

static void update_vel_cmd(uint8_t ch)
{
    if ( floats_equal(g.vel_cmd, gp.vel_cmd_old) ) return;

    gp.step_freq_new = E * (hal_s64_t)(g.vel_cmd * g.pos_scale);
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

    // input
    #define p_scale     g.pos_scale
    #define a_max       g.accel_max
    #define v_max       g.vel_max
    #define p_cmd       g.pos_cmd
    #define v_cmd       g.vel_cmd
    // output
    #define p           g.pos_fb
    #define p_prev      gp.pos_fb_prev
    #define counts      g.counts
    #define counts_prev gp.counts_prev
    #define freq        g.freq
    // other vars
    double a_now;
    double d_dist;
    int dir;
    int goto_dir;
    int a_dir;
    int v_dir;
    double v_tmp1;
    double v_tmp2;
    #define v           gp.vel
    #define v_prev      gp.vel_prev
    // actions
    int steps = 0;
    int dirs = 0;

    for ( ch = sg_cnt; ch--; )
    {
        // channel disabled?
        if ( !g.enable )
        {
            // are we still moving?
            if ( freq )
            {
//                gp.step_freq = 0;
                freq = 0;
                stepgen_abort(ch, 1); // abort all tasks
            }

            continue;
        }

        // recalculate private and public data
//        update_stepdir_time(ch);
        update_pins(ch);
        update_pos_scale(ch);
//        update_accel_max(ch);
//        update_vel_max(ch);

        v_prev      = v;
        p_prev      = p;
        counts_prev = counts;

        // are we using a velocity mode?
        if ( gp.ctrl_type )
        {
            a_now   = a_max != 0 ? a_max*period/1000000000 : rtapi_fabs(v_prev - v_cmd);
            v_dir   = v_cmd > v_prev ? 1 : -1;
            v_tmp1  = v_prev + v_dir*a_now;
            v_tmp2  = (v_dir > 0 ? v_tmp1 > v_cmd : v_tmp1 < v_cmd) ? v_cmd : v_tmp1;
            v       = (v_max != 0) && rtapi_fabs(v_tmp2) > rtapi_fabs(v_max) ?
                        (v_tmp2 >= 0 ? 1 : -1)*v_max :
                        v_tmp2;
        }
        else // position mode
        {
            a_now       = a_max * period / 1000000000;
            dir         = v_prev >= 0 ? 1 : -1;
            d_dist      = a_max != 0 ? ((rtapi_fabs(v_prev+a_now*dir))^2) / (2*a_max) : 0;
            goto_dir    = p_cmd >= p_prev ? 1 : -1;
            a_dir       = rtapi_fabs(p_cmd - p_prev) > d_dist ? 1 : -1;
            v_dir       = dir != goto_dir ? goto_dir : dir * a_dir;
            v_tmp1      = a_max != 0 ? v_prev + a_now*v_dir : (p_cmd - p_prev)*1000000000/period;
            v_tmp2      = v_max != 0 && rtapi_fabs(v_tmp1) > v_max ? (v_tmp1 >= 0 ? 1 : -1)*v_max : v_tmp1;
            v           = abs(round(p_cmd*p_scale) - counts_prev) > 0 ?
                              v_tmp2 :
                              (a_max != 0 && rtapi_fabs(v_prev) > 1.1*a_now ? v_tmp2 : 0);
        }

        p = p_prev + v * period / 1000000000;
        counts = round( p * p_scale );
        freq = rtapi_fabs(v * p_scale);

        steps = abs(counts - counts_prev);
        dirs = (v_prev >= 0 && v >= 0) || (v_prev < 0 && v < 0) ? 0 : (v != 0 ? 1 : 0);

        // TODO
    }

    #undef p_scale
    #undef a_max
    #undef v_max
    #undef p_cmd
    #undef v_cmd
    #undef p
    #undef p_prev
    #undef counts
    #undef counts_prev
    #undef freq
    #undef v
    #undef v_prev
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

        gp.counts_prev = 0;
        gp.pos_fb_prev = 0.0;
        gp.vel_prev = 0.0;
        gp.vel = 0.0;
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
