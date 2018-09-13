/**
 * @file    stepgen_new.h
 * @brief   STEPGEN related data
 * @author  Mikhail Vydrenko (mikhail@vydrenko.ru)
 */

#ifndef _STEPGEN_H
#define _STEPGEN_H

#include "rtapi.h"
#include "rtapi_app.h"
#include "hal.h"
#include <float.h>
#include <stdlib.h>
#include "rtapi_math.h"

#include "gpio.h"
#include "pulsgen.h"




// DATA

#define STEPGEN_CH_CNT_MAX 10

#define g *sg[ch]
#define gp sg[ch]

#define EXPORT_PIN(IO_TYPE,VAR_TYPE,VAL,NAME,DEFAULT) \
    r += hal_pin_##VAR_TYPE##_newf(IO_TYPE, &(gp->VAL), comp_id,\
    "%s.stepgen.%d." NAME, comp_name, ch);\
    g->VAL = DEFAULT;

#define EXPORT_FUNC(FUNC,NAME,FP) \
    hal_export_functf(FUNC, 0, FP, 0, comp_id, "%s.stepgen."NAME, comp_name)

#define PRINT_ERROR(MSG) \
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: [STEPGEN] "MSG"\n", comp_name)

#define PRINT_ERROR_AND_RETURN(MSG,RETVAL) \
    { PRINT_ERROR(MSG); return RETVAL; }

enum
{
    TASK_IDLE = 0,
    TASK_FWD,
    TASK_REV,
    TASK_DIR_FWD,
    TASK_DIR_REV,
    TASK_FWD_DIR_REV,
    TASK_REV_DIR_FWD
};

typedef struct
{
    // available for all functions

    hal_bit_t *enable;
    hal_u32_t *step_space;
    hal_u32_t *step_len;
    hal_u32_t step_space_old; // private
    hal_u32_t step_len_old; // private
    hal_u32_t *dir_setup;
    hal_u32_t *dir_hold;
    hal_s32_t *counts;
    hal_s32_t *rawcounts;
    hal_u32_t *step_port;
    hal_u32_t *step_pin;
    hal_bit_t *step_inv;
    hal_u32_t step_port_old; // private
    hal_u32_t step_pin_old; // private
    hal_bit_t step_inv_old; // private
    hal_u32_t step_pulsgen_ch0; // private
    hal_u32_t step_pulsgen_ch1; // private
    hal_bit_t step_ch_ready; // private
    hal_u32_t *dir_port;
    hal_u32_t *dir_pin;
    hal_bit_t *dir_inv;
    hal_u32_t dir_port_old; // private
    hal_u32_t dir_pin_old; // private
    hal_bit_t dir_inv_old; // private
    hal_u32_t dir_pulsgen_ch; // private
    hal_bit_t dir_ch_ready; // private

    hal_u32_t steps_freq; // private
    hal_u32_t steps_freq_max; // private
    hal_u32_t steps_accel_max; // private

    hal_bit_t task; // private
    hal_s32_t task_counts; // private

    // aren't available for the `make_pulses()`

    hal_float_t *vel_max;
    hal_float_t *accel_max;
    hal_float_t vel_max_old; // private
    hal_float_t accel_max_old; // private
    hal_float_t *pos_scale;
    hal_float_t *pos_cmd;
    hal_float_t pos_cmd_old; // private
    hal_float_t *pos_fb;
    hal_float_t *freq;
} stepgen_ch_t;









// VAR

static int8_t *num_chan = "0";
RTAPI_MP_STRING(num_chan, "Channels count");

static stepgen_ch_t *sg;
static uint8_t ch_cnt = 0;









// TOOLS

static hal_bit_t
floats_equal(hal_float_t f1, hal_float_t f2)
{
    return *((int64_t*) &f1) == *((int64_t*) &f2);
}

static hal_bit_t
step_state_get(uint8_t ch)
{
    return g->step_inv ^ gpio_pin_get(g->step_port, g->step_pin) ? 0 : 1;
}

static hal_bit_t
dir_state_get(uint8_t ch)
{
    return g->dir_inv ^ gpio_pin_get(g->dir_port, g->dir_pin) ? 0 : 1;
}

static void
step_state_set(uint8_t ch, hal_bit_t state)
{
    if ( g->step_inv ^ state ) gpio_pin_clear (g->step_port, g->step_pin);
    else                       gpio_pin_set   (g->step_port, g->step_pin);
}

static void
dir_state_set(uint8_t ch, hal_bit_t state)
{
    if ( g->dir_inv ^ state )  gpio_pin_clear (g->dir_port, g->dir_pin);
    else                       gpio_pin_set   (g->dir_port, g->dir_pin);
}

static void
abort_output(uint8_t ch)
{
    pulsgen_task_abort(gp->step_pulsgen_ch0);
    pulsgen_task_abort(gp->step_pulsgen_ch1);
    pulsgen_task_abort(gp->dir_pulsgen_ch);
}

static void
abort_task(uint8_t ch)
{
    gp->task        = TASK_IDLE;
    gp->steps_freq  = 0;
    gp->freq        = 0;
}

static void
setup_step_ch(uint8_t ch)
{
    if ( g->step_port < GPIO_PORTS_CNT && g->step_pin < GPIO_PINS_CNT )
    {
        gp->step_ch_ready = 1;
        pulsgen_pin_setup(gp->step_pulsgen_ch0, g->step_port, g->step_pin, g->step_inv);
        pulsgen_pin_setup(gp->step_pulsgen_ch1, g->step_port, g->step_pin, g->step_inv);
    }
    else gp->step_ch_ready = 0;
}

static void
setup_dir_ch(uint8_t ch)
{
    if ( g->dir_port < GPIO_PORTS_CNT && g->dir_pin < GPIO_PINS_CNT )
    {
        gp->dir_ch_ready = 1;
        pulsgen_pin_setup(gp->dir_pulsgen_ch, g->dir_port, g->dir_pin, g->dir_inv);
    }
    else gp->dir_ch_ready = 0;
}

static hal_bit_t
step_pin_changed(uint8_t ch)
{
    return ( g->step_port != gp->step_port_old  ||
             g->step_pin  != gp->step_pin       ||
             g->step_inv  != gp->step_inv ) ? 1 : 0;
}

static hal_bit_t
dir_pin_changed(uint8_t ch)
{
    return ( g->dir_port != gp->dir_port_old    ||
             g->dir_pin  != gp->dir_pin         ||
             g->dir_inv  != gp->dir_inv ) ? 1 : 0;
}

static void
save_step_pin(uint8_t ch)
{
    gp->step_port_old   = g->step_port;
    gp->step_pin        = g->step_pin;
    gp->step_inv        = g->step_inv;
}

static void
save_dir_pin(uint8_t ch)
{
    gp->dir_port_old    = g->dir_port;
    gp->dir_pin         = g->dir_pin;
    gp->dir_inv         = g->dir_inv;
}

static hal_s32_t
get_rawcounts(uint8_t ch)
{
    if ( !gp->task ) return g->rawcounts;

    hal_s32_t counts = g->rawcounts;

    switch ( gp->task )
    {
        case TASK_FWD:
            counts += pulsgen_task_toggles(gp->step_pulsgen_ch0) / 2;
            break;

        case TASK_REV:
            counts -= pulsgen_task_toggles(gp->step_pulsgen_ch0) / 2;
            break;

        case TASK_DIR_FWD:
            hal_u32_t step_toggles0 = pulsgen_task_toggles(gp->step_pulsgen_ch0);
            hal_u32_t dir_toggles   = pulsgen_task_toggles(gp->dir_pulsgen_ch);
            counts += (dir_toggles ? step_toggles0 : -step_toggles0) / 2;
            break;

        case TASK_DIR_REV:
            hal_u32_t step_toggles0 = pulsgen_task_toggles(gp->step_pulsgen_ch0);
            hal_u32_t dir_toggles   = pulsgen_task_toggles(gp->dir_pulsgen_ch);
            counts -= (dir_toggles ? step_toggles0 : -step_toggles0) / 2;
            break;

        case TASK_FWD_DIR_REV:
            hal_u32_t step_toggles0 = pulsgen_task_toggles(gp->step_pulsgen_ch0);
            hal_u32_t step_toggles1 = pulsgen_task_toggles(gp->step_pulsgen_ch1);
            hal_u32_t dir_toggles   = pulsgen_task_toggles(gp->dir_pulsgen_ch);
            counts += step_toggles0 / 2;
            counts -= (dir_toggles ? step_toggles1 : -step_toggles1) / 2;
            break;

        case TASK_REV_DIR_FWD:
            hal_u32_t step_toggles0 = pulsgen_task_toggles(gp->step_pulsgen_ch0);
            hal_u32_t step_toggles1 = pulsgen_task_toggles(gp->step_pulsgen_ch1);
            hal_u32_t dir_toggles   = pulsgen_task_toggles(gp->dir_pulsgen_ch);
            counts -= step_toggles0 / 2;
            counts += (dir_toggles ? step_toggles1 : -step_toggles1) / 2;
            break;
    }

    return counts;
}









// HAL functions

static void stepgen_capture_pos(void *arg, long period)
{
    // TODO:
    // +++  if POS_SCALE == 0.0 then POS_SCALE = 1.0
    // +++  capture position in steps (counts, rawcounts)
    // +++  capture position in units

    static uint8_t ch;

    for ( ch = ch_cnt; ch--; )
    {
        // if POS_SCALE == 0.0 then POS_SCALE = 1.0
        if ( g->pos_scale < 1e-20 && g->pos_scale > -1e-20 ) g->pos_scale = 1.0;

        // capture position in steps (counts, rawcounts)
        g->rawcounts = get_rawcounts(ch);
        g->counts    = g->rawcounts;

        // capture position in units
        g->pos_fb = ((hal_float_t)g->counts) / g->pos_scale;
    }
}

static void stepgen_update_freq(void *arg, long period)
{
    // TODO:
    // +++  abort output if channel is disabled
    // +++  recalculate private and public data
    // +++  capture position in steps (counts, rawcounts)
    //      start output
    //      update frequency and continue output
    //      stop output if in position

    static uint8_t ch;

    for ( ch = ch_cnt; ch--; )
    {
        // abort output if channel is disabled
        if ( gp->task && !g->enable ) abort_output();

        // capture position in steps (counts, rawcounts)
        g->rawcounts = get_rawcounts(ch);
        g->counts    = g->rawcounts;

        // abort task
        if ( gp->task && !g->enable ) { abort_task(); continue; }

        // setup pulsgen channels
        if ( step_pin_changed(ch) ) { setup_step_ch(ch); save_step_pin(ch); }
        if ( dir_pin_changed(ch) )  { setup_dir_ch(ch);  save_dir_pin(ch); }

        // recalculate private and public data

        if ( !floats_equal(g->accel_max, gp->accel_max_old) )
        {
            gp->steps_accel_max = (hal_u32_t) (g->accel_max * g->pos_scale);
            gp->accel_max_old   = g->accel_max;
        }

        if ( !floats_equal(g->vel_max, gp->vel_max_old) )
        {
            gp->steps_freq_max  = (hal_u32_t) (g->vel_max * g->pos_scale);
            gp->vel_max_old     = g->vel_max;
        }

        if ( g->step_len != gp->step_len_old || g->step_space != gp->step_space_old )
        {
            hal_u32_t freq_max = 1000000000 / (g->step_len + g->step_space);
            if ( gp->steps_freq_max > freq_max ) gp->steps_freq_max = freq_max;

            gp->step_len_old    = g->step_len;
            gp->step_space_old  = g->step_space
        }
    }
}

static void stepgen_make_pulses(void *arg, long period)
{
    // TODO:
    // +++  abort output if channel is disabled
    // +++  capture position in steps (rawcounts)

    static uint8_t ch;

    for ( ch = ch_cnt; ch--; )
    {
        // abort output if channel is disabled
        // capture position in steps (rawcounts)

        if ( gp->task && !g->enable ) abort_output();

        g->rawcounts = get_rawcounts(ch);

        if ( gp->task && !g->enable ) abort_task();
    }
}








// INIT

static int32_t stepgen_malloc_and_export(const char *comp_name, int32_t comp_id)
{
    int32_t r, ch, pulsgen_ch;

    // get num_chan count

    ch_cnt = (uint8_t) strtoul((const char *)num_chan, NULL, 10);
    if ( !ch_cnt ) return 0;
    if ( ch_cnt > STEPGEN_CH_CNT_MAX ) ch_cnt = STEPGEN_CH_CNT_MAX;

    // shared memory allocation

    sg = hal_malloc(ch_cnt * sizeof(stepgen_ch_t));
    if ( !sg ) PRINT_ERROR_AND_RETURN("hal_malloc() failed", -1);

    // export HAL pins and set default values

    for ( r = 0, pulsgen_ch = 0, ch = ch_cnt; ch--; )
    {
        EXPORT_PIN(HAL_IN,bit,enable,"enable", 0);
        EXPORT_PIN(HAL_IN,u32,step_space,"stepspace", 1000);
        EXPORT_PIN(HAL_IN,u32,step_len,"steplen", 1000);
        EXPORT_PIN(HAL_IN,u32,dir_setup,"dirsetup", 1000);
        EXPORT_PIN(HAL_IN,u32,dir_hold,"dirhold", 1000);
        EXPORT_PIN(HAL_IN,u32,step_port,"step-port", UINT32_MAX);
        EXPORT_PIN(HAL_IN,u32,step_pin,"step-pin", UINT32_MAX);
        EXPORT_PIN(HAL_IN,bit,step_inv,"step-inv", 0);
        EXPORT_PIN(HAL_IN,u32,dir_port,"dir-port", UINT32_MAX);
        EXPORT_PIN(HAL_IN,u32,dir_pin,"dir-pin", UINT32_MAX);
        EXPORT_PIN(HAL_IN,bit,dir_inv,"dir-inv", 0);
        EXPORT_PIN(HAL_IN,float,pos_scale,"position-scale", 1.0);
        EXPORT_PIN(HAL_IN,float,vel_max,"maxvel", 0.0);
        EXPORT_PIN(HAL_IN,float,accel_max,"maxaccel", 0.0);
        EXPORT_PIN(HAL_IN,float,pos_cmd,"position-cmd", 0.0);
        EXPORT_PIN(HAL_OUT,float,pos_fb,"position-fb", 0.0);
        EXPORT_PIN(HAL_OUT,float,freq,"frequency", 0.0);
        EXPORT_PIN(HAL_OUT,s32,counts,"counts", 0);
        EXPORT_PIN(HAL_OUT,s32,rawcounts,"rawcounts", 0);

        g->step_pulsgen_ch0 = pulsgen_ch++;
        g->step_pulsgen_ch1 = pulsgen_ch++;
        g->dir_pulsgen_ch = pulsgen_ch++;

        g->step_inv_old = 0;
        g->step_pin_old = UINT32_MAX;
        g->step_port_old = UINT32_MAX;
        g->step_len_old = 1000;
        g->step_space_old = 1000;
        g->step_ch_ready = 0;
        g->dir_inv_old = 0;
        g->dir_pin_old = UINT32_MAX;
        g->dir_port_old = UINT32_MAX;
        g->dir_ch_ready = 0;
        g->pos_cmd_old = 0.0;
        g->vel_max_old = 0.0;
        g->accel_max_old = 0.0;
        g->steps_freq = 0;
        g->steps_freq_max = 0;
        g->steps_accel_max = 0;

        g->task = 0;
        g->task_counts = 0;
    }
    if ( r ) PRINT_ERROR_AND_RETURN("HAL pins export failed", -1);

    // export HAL functions

    r = 0;
    r += EXPORT_FUNC(stepgen_capture_pos, "capture-position", 1);
    r += EXPORT_FUNC(stepgen_update_freq, "update-freq", 1);
    r += EXPORT_FUNC(stepgen_make_pulses, "make-pulses", 0);
    if ( r ) PRINT_ERROR_AND_RETURN("HAL functions export failed", -1);

    return 0;
}




#undef EXPORT_PIN
#undef PRINT_ERROR
#undef PRINT_ERROR_AND_RETURN
#undef g
#undef gp

#endif
