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




// TOOLS
#if 0
static hal_bit_t sg_floats_equal(hal_float_t f1, hal_float_t f2)
{
    return *((int64_t*) &f1) == *((int64_t*) &f2);
}

static hal_bit_t sg_step_state_get(uint8_t ch)
{
    if ( !gp.step_ch_ready ) return 0;

    return g.step_inv ^ gpio_pin_get(g.step_port, g.step_pin) ? 0 : 1;
}

static hal_bit_t sg_dir_state_get(uint8_t ch)
{
    if ( !gp.dir_ch_ready ) return 0;

    return g.dir_inv ^ gpio_pin_get(g.dir_port, g.dir_pin) ? 0 : 1;
}

static void sg_step_state_set(uint8_t ch, hal_bit_t state)
{
    if ( !gp.step_ch_ready ) return;

    if ( g.step_inv ^ state )   gpio_pin_clear (g.step_port, g.step_pin);
    else                        gpio_pin_set   (g.step_port, g.step_pin);
}

static void sg_dir_state_set(uint8_t ch, hal_bit_t state)
{
    if ( !gp.dir_ch_ready ) return;

    if ( g.dir_inv ^ state )    gpio_pin_clear (g.dir_port, g.dir_pin);
    else                        gpio_pin_set   (g.dir_port, g.dir_pin);
}

static void sg_abort_output(uint8_t ch)
{
    if ( gp.step_ch_ready )
    {
        pulsgen_task_abort(gp.step_pulsgen_ch0, 0);
        pulsgen_task_abort(gp.step_pulsgen_ch1, 0);
    }
    if ( gp.dir_ch_ready )
    {
        pulsgen_task_abort(gp.dir_pulsgen_ch, 0);
    }
}

static void sg_idle(uint8_t ch)
{
    gp.pulsgen_task     = TASK_IDLE;
    gp.step_freq        = 0;
    gp.step_wait_time   = 0;
    gp.step_freq_new    = 0;
    gp.counts_new       = g.counts;
}

static void sg_update_accel_max(uint8_t ch)
{
    if ( sg_floats_equal(g.accel_max, gp.accel_max_old) ) return;
    if ( g.accel_max < 0 ) g.accel_max = 1.0;

    gp.step_accel_max   = (hal_u32_t) (g.accel_max * g.pos_scale);
    gp.accel_max_old    = g.accel_max;
}

static void sg_update_pos_scale(uint8_t ch)
{
    if ( g.pos_scale < 0 || (g.pos_scale < 1e-20 && g.pos_scale > -1e-20) ) g.pos_scale = 1.0;
}

static void sg_update_vel_max(uint8_t ch)
{
    if ( !sg_floats_equal(g.vel_max, gp.vel_max_old) )
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

static void sg_update_vel_cmd(uint8_t ch)
{
    if ( sg_floats_equal(g.vel_cmd, gp.vel_cmd_old) ) return;

    gp.step_freq_new = (hal_s32_t) (g.vel_cmd * g.pos_scale);
    gp.vel_cmd_old = g.vel_cmd;
}

static void sg_update_pos_cmd(uint8_t ch)
{
    if ( sg_floats_equal(g.pos_cmd, gp.pos_cmd_old) ) return;

    gp.counts_new = (hal_s32_t) (g.pos_cmd * g.pos_scale);
    gp.pos_cmd_old = g.pos_cmd;
}

static void sg_update_pins(uint8_t ch)
{
    if ( g.step_port != gp.step_port_old    ||
         g.step_pin  != gp.step_pin_old     ||
         g.step_inv  != gp.step_inv_old )
    {
        if ( g.step_port < GPIO_PORTS_CNT && g.step_pin < GPIO_PINS_CNT )
        {
            gp.step_ch_ready = 1;
            pulsgen_pin_setup(gp.step_pulsgen_ch0, g.step_port, g.step_pin, g.step_inv);
            pulsgen_pin_setup(gp.step_pulsgen_ch1, g.step_port, g.step_pin, g.step_inv);
        }
        else gp.step_ch_ready = 0;

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
            gp.dir_ch_ready = 1;
            pulsgen_pin_setup(gp.dir_pulsgen_ch, g.dir_port, g.dir_pin, g.dir_inv);
        }
        else gp.dir_ch_ready = 0;

        gp.dir_port_old = g.dir_port;
        gp.dir_pin_old  = g.dir_pin;
        gp.dir_inv_old  = g.dir_inv;
    }
}

static void sg_update_stepdir_time(uint8_t ch)
{
    if ( !g.step_len )   { g.step_len   = 5000;  gp.step_len_old   = g.step_len; }
    if ( !g.step_space ) { g.step_space = 5000;  gp.step_space_old = g.step_space; }

    if ( !g.dir_setup ) g.dir_setup = 35000;
    if ( !g.dir_hold )  g.dir_hold  = 35000;

    if ( (g.dir_hold + g.dir_setup) > STEPGEN_DIR_TIME_MAX )
    {
        g.dir_setup = STEPGEN_DIR_TIME_MAX / 2;
        g.dir_hold  = STEPGEN_DIR_TIME_MAX / 2;
    }
}

static void sg_update_counts(uint8_t ch)
{
    if ( !gp.pulsgen_task ) return;

    uint32_t step_toggles0, step_toggles1, dir_toggles;

    switch ( gp.pulsgen_task )
    {
        case TASK_FWD:
            g.rawcounts += pulsgen_task_toggles(gp.step_pulsgen_ch0) / 2;
            break;

        case TASK_REV:
            g.rawcounts -= pulsgen_task_toggles(gp.step_pulsgen_ch0) / 2;
            break;

        case TASK_DIR_FWD:
            step_toggles0 = pulsgen_task_toggles(gp.step_pulsgen_ch0);
            dir_toggles   = pulsgen_task_toggles(gp.dir_pulsgen_ch);
            g.rawcounts += (dir_toggles ? step_toggles0 : -step_toggles0) / 2;
            break;

        case TASK_DIR_REV:
            step_toggles0 = pulsgen_task_toggles(gp.step_pulsgen_ch0);
            dir_toggles   = pulsgen_task_toggles(gp.dir_pulsgen_ch);
            g.rawcounts -= (dir_toggles ? step_toggles0 : -step_toggles0) / 2;
            break;

        case TASK_FWD_DIR_REV:
            step_toggles0 = pulsgen_task_toggles(gp.step_pulsgen_ch0);
            step_toggles1 = pulsgen_task_toggles(gp.step_pulsgen_ch1);
            dir_toggles   = pulsgen_task_toggles(gp.dir_pulsgen_ch);
            g.rawcounts += step_toggles0 / 2;
            g.rawcounts -= (dir_toggles ? step_toggles1 : -step_toggles1) / 2;
            break;

        case TASK_REV_DIR_FWD:
            step_toggles0 = pulsgen_task_toggles(gp.step_pulsgen_ch0);
            step_toggles1 = pulsgen_task_toggles(gp.step_pulsgen_ch1);
            dir_toggles   = pulsgen_task_toggles(gp.dir_pulsgen_ch);
            g.rawcounts -= step_toggles0 / 2;
            g.rawcounts += (dir_toggles ? step_toggles1 : -step_toggles1) / 2;
            break;
    }

    g.counts = g.rawcounts;
}

static void sg_abort_all(uint8_t ch)
{
    sg_abort_output(ch);
    sg_update_counts(ch);
    sg_idle(ch);
}

static void sg_setup_pulsgen(uint8_t ch, long period)
{
    static hal_s32_t step_period, steps, step_hold, step_space, delay;

    if ( gp.ctrl_type ) // velocity control type
    {
        if ( gp.step_wait_time >= (period - g.step_len) )
        {
            gp.step_wait_time -= period;
        }
        else
        {
            step_period = gp.step_freq ? 1000000000 / abs(gp.step_freq) : 0;
            delay = gp.step_wait_time < 0 ? 0 : gp.step_wait_time;

            if ( g.step_len > step_period )
            {
                step_hold = step_period / 2;
                step_space = step_hold;
            }
            else
            {
                step_hold = g.step_len;
                step_space = step_period - g.step_len;
            }

            switch ( gp.pulsgen_task )
            {
                case TASK_FWD:
                case TASK_REV:
                    steps = 1 + (period - gp.step_wait_time - g.step_len) / step_period;
                    steps = steps < 0 ? 0 : steps;

                    if ( gp.step_ch_ready && step_hold && step_space && steps )
                        pulsgen_task_setup(gp.step_pulsgen_ch0, 2*steps, step_space, step_hold, delay);

                    break;

                case TASK_DIR_FWD:
                case TASK_DIR_REV:
                    period -= g.dir_hold;
                    steps = 1 + (period - gp.step_wait_time - g.step_len) / step_period;
                    steps = steps < 0 ? 0 : steps;

                    if ( gp.dir_ch_ready )
                        pulsgen_task_setup(gp.dir_pulsgen_ch, 1, 1, 1, 0);

                    if ( gp.step_ch_ready && step_hold && step_space && steps )
                        pulsgen_task_setup(gp.step_pulsgen_ch0, 2*steps, step_space, step_hold, delay + g.dir_hold);

                    break;

                case TASK_REV_DIR_FWD:
                case TASK_FWD_DIR_REV:
                    // TODO
                    break;
            }

            gp.step_wait_time += steps * step_period - period;
        }

        return;
    }

    // position mode
    // TODO
}
#endif



// HAL functions

static void sg_capture_pos(void *arg, long period)
{
#if 0
    static uint8_t ch;

    for ( ch = sg_cnt; ch--; )
    {
        if ( !g.enable ) continue;
        // if POS_SCALE == 0.0 then POS_SCALE = 1.0
        sg_update_pos_scale(ch);

        // capture position in steps (counts, rawcounts)
        sg_update_counts(ch);

        // capture position in units
        g.pos_fb = ((hal_float_t)g.counts) / g.pos_scale;
    }
#endif
}

static void sg_update_freq(void *arg, long period)
{
#if 0
    static uint8_t ch, wd = 0;
    static int8_t dir_old, dir_new, dir_cur;
    static hal_s64_t freq;

    // enable watchdog with `512*period` timeout
    // and update watchdog every `256*period` time
    if ( !wd ) pulsgen_watchdog_setup(1, 512*period);
    wd++;

    for ( ch = sg_cnt; ch--; )
    {
        if ( !g.enable )
        {
            if ( gp.pulsgen_task ) sg_abort_all(ch);
            g.freq = 0.0;
            continue;
        }

        // recalculate private and public data
        sg_update_stepdir_time(ch);
        sg_update_pins(ch);
        sg_update_pos_scale(ch);
        sg_update_accel_max(ch);
        sg_update_vel_max(ch);

        // abort output
        if ( gp.pulsgen_task ) sg_abort_output(ch);

        sg_update_counts(ch);

        if ( gp.ctrl_type ) // velocity control type
        {
            sg_update_vel_cmd(ch);

            freq = gp.step_freq;
            dir_old = gp.step_freq >= 0 ? 1 : -1;
            dir_new = gp.step_freq_new >= 0 ? 1 : -1;

            if ( gp.step_freq != gp.step_freq_new )
            {
                if ( gp.step_accel_max )
                {
                    freq += dir_new * period * gp.step_accel_max / 1000000000;

                    if ( freq >= 0 && gp.step_freq_new >= 0 && freq > gp.step_freq_new ) freq = gp.step_freq_new;
                    else if ( freq < 0 && gp.step_freq_new < 0 && freq < gp.step_freq_new ) freq = gp.step_freq_new;
                }
                else freq = gp.step_freq_new;

                if ( gp.step_freq_max )
                {
                    if ( freq >= 0 && freq > gp.step_freq_max ) freq = gp.step_freq_max;
                    else if ( freq < 0 && freq < -gp.step_freq_max ) freq = -gp.step_freq_max;
                }

                if      ( freq > INT32_MAX ) freq = INT32_MAX;
                else if ( freq < INT32_MIN ) freq = INT32_MIN;
            }

            if ( freq )
            {
                if ( gp.step_freq )
                {
                    dir_new = freq >= 0 ? 1 : -1;
                    if ( dir_old == dir_new ) gp.pulsgen_task = dir_old == -1 ? TASK_REV         : TASK_FWD;
                    else                      gp.pulsgen_task = dir_new == -1 ? TASK_FWD_DIR_REV : TASK_REV_DIR_FWD;
                }
                else
                {
                    if ( dir_old == dir_new ) gp.pulsgen_task = dir_old == -1 ? TASK_REV     : TASK_FWD;
                    else                      gp.pulsgen_task = dir_new == -1 ? TASK_DIR_REV : TASK_DIR_FWD;
                }
            }
            else gp.pulsgen_task = TASK_IDLE;

            gp.step_freq = (hal_s32_t) freq;
        }
        else // position control type
        {
            sg_update_pos_cmd(ch);

            // TODO
        }

        if ( gp.step_freq && gp.pulsgen_task ) sg_setup_pulsgen(ch, period);

        g.freq = ((hal_float_t) gp.step_freq) / g.pos_scale;
    }
#endif
}

static void sg_make_pulses(void *arg, long period) // no floats here!
{
#if 0
    static uint8_t ch;

    for ( ch = sg_cnt; ch--; )
    {
        if ( !gp.pulsgen_task ) continue;

        if ( g.enable ) sg_update_counts(ch);
        else            sg_abort_all(ch);
    }
#endif
}





// INIT

static int32_t sg_malloc_and_export(const char *comp_name, int32_t comp_id)
{
    int32_t r, ch, pulsgen_ch;
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

    for ( r = 0, pulsgen_ch = 0, ch = sg_cnt; ch--; )
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

        gp.step_pulsgen_ch0 = pulsgen_ch++;
        gp.step_pulsgen_ch1 = pulsgen_ch++;
        gp.dir_pulsgen_ch = pulsgen_ch++;

        gp.counts_new = 0;
        gp.pulsgen_task = 0;
        gp.ctrl_type = type[ch];

        gp.step_inv_old = 0;
        gp.step_pin_old = UINT32_MAX;
        gp.step_port_old = UINT32_MAX;
        gp.step_len_old = 1000;
        gp.step_space_old = 1000;
        gp.step_ch_ready = 0;
        gp.step_freq = 0;
        gp.step_wait_time = 0;
        gp.step_freq_new = 0;
        gp.step_freq_max = 0;
        gp.step_accel_max = 0;

        gp.dir_inv_old = 0;
        gp.dir_pin_old = UINT32_MAX;
        gp.dir_port_old = UINT32_MAX;
        gp.dir_ch_ready = 0;

        gp.vel_max_old = 0.0;
        gp.accel_max_old = 0.0;
    }
    if ( r ) PRINT_ERROR_AND_RETURN("HAL pins export failed", -1);

    // export HAL functions

    r = 0;
    r += EXPORT_FUNC(sg_capture_pos, "capture-position", 1);
    r += EXPORT_FUNC(sg_update_freq, "update-freq", 1);
    r += EXPORT_FUNC(sg_make_pulses, "make-pulses", 0);
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
    if ( sg_malloc_and_export(comp_name, comp_id) ) EXIT;

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
