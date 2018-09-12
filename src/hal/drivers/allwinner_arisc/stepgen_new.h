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




#define STEPGEN_CH_CNT_MAX 10

#define PRINT_ERROR(MSG) \
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: [STEPGEN] "MSG"\n", comp_name)

#define PRINT_ERROR_AND_RETURN(MSG,RETVAL) \
    { PRINT_ERROR(MSG); return RETVAL; }

typedef struct
{
    // available for all functions
    hal_bit_t *enable;
    hal_u32_t *step_space;
    hal_u32_t *step_len;
    hal_u32_t *dir_setup;
    hal_u32_t *dir_hold;
    hal_s32_t *counts;
    hal_s32_t *rawcounts;
    hal_u32_t *step_port;
    hal_u32_t *step_pin;
    hal_bit_t *step_inv;
    hal_u32_t step_pulsgen_ch0;
    hal_u32_t step_pulsgen_ch1;
    hal_u32_t *dir_port;
    hal_u32_t *dir_pin;
    hal_bit_t *dir_inv;
    hal_u32_t dir_pulsgen_ch;

    // aren't available for the `make_pulses()`
    hal_float_t *vel_max;
    hal_float_t *accel_max;
    hal_float_t *pos_scale;
    hal_float_t *pos_cmd;
    hal_float_t *pos_fb;
    hal_float_t *freq;
} stepgen_ch_t;









static int8_t *num_chan = "0";
RTAPI_MP_STRING(num_chan, "Channels count");

static stepgen_ch_t *sg;
static uint8_t ch_cnt = 0;









static void stepgen_capture_pos(void *arg, long period)
{
}

static void stepgen_update_freq(void *arg, long period)
{
}

static void stepgen_make_pulses(void *arg, long period)
{
}




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

#define EXPORT_PIN(IO_TYPE,TYPE,VAL,NAME,DEFAULT) \
    r += hal_pin_##TYPE##_newf(IO_TYPE, &(sg[ch].VAL), comp_id,\
    "%s.stepgen.%d." NAME, comp_name, ch);\
    *sg[ch].VAL = DEFAULT;

    for ( r = 0, pulsgen_ch = 0, ch = ch_cnt; ch--; )
    {
        EXPORT_PIN(HAL_IN,bit,enable,"enable", 0);
        EXPORT_PIN(HAL_IN,u32,step_space,"stepspace", 1);
        EXPORT_PIN(HAL_IN,u32,step_len,"steplen", 1);
        EXPORT_PIN(HAL_IN,u32,dir_setup,"dirsetup", 1);
        EXPORT_PIN(HAL_IN,u32,dir_hold,"dirhold", 1);
        EXPORT_PIN(HAL_IN,u32,step_port,"step-port", PA);
        EXPORT_PIN(HAL_IN,u32,step_pin,"step-pin", 15);
        EXPORT_PIN(HAL_IN,bit,step_inv,"step-inv", 0);
        EXPORT_PIN(HAL_IN,u32,dir_port,"dir-port", PL);
        EXPORT_PIN(HAL_IN,u32,dir_pin,"dir-pin", 10);
        EXPORT_PIN(HAL_IN,bit,dir_inv,"dir-inv", 0);
        EXPORT_PIN(HAL_IN,float,pos_scale,"position-scale", 1.0);
        EXPORT_PIN(HAL_IN,float,vel_max,"maxvel", 0.0);
        EXPORT_PIN(HAL_IN,float,accel_max,"maxaccel", 0.0);
        EXPORT_PIN(HAL_IN,float,pos_cmd,"position-cmd", 0.0);
        EXPORT_PIN(HAL_OUT,float,pos_fb,"position-fb", 0.0);
        EXPORT_PIN(HAL_OUT,float,freq,"frequency", 0.0);
        EXPORT_PIN(HAL_OUT,s32,counts,"counts", 0);
        EXPORT_PIN(HAL_OUT,s32,rawcounts,"rawcounts", 0);

        sg[ch]->step_pulsgen_ch0 = pulsgen_ch++;
        sg[ch]->step_pulsgen_ch1 = pulsgen_ch++;
        sg[ch]->dir_pulsgen_ch = pulsgen_ch++;
    }

    if ( r ) PRINT_ERROR_AND_RETURN("HAL pins export failed", -1);

#undef EXPORT_PIN

    // export HAL functions

    r = 0;
    r+= hal_export_functf(stepgen_capture_pos, 0, 1, 0, comp_id, "%s.stepgen.capture-position", comp_name);
    r+= hal_export_functf(stepgen_update_freq, 0, 1, 0, comp_id, "%s.stepgen.update-freq", comp_name);
    r+= hal_export_functf(stepgen_make_pulses, 0, 0, 0, comp_id, "%s.stepgen.make-pulses", comp_name);
    if ( r ) PRINT_ERROR_AND_RETURN("HAL functions export failed", -1);

    return 0;
}




#endif
