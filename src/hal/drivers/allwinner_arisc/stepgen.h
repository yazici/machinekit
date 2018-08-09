/**
 * @file    stepgen.h
 * @brief   STEPGEN related data
 * @author  Mikhail Vydrenko (mikhail@vydrenko.ru)
 */

#ifndef _STEPGEN_H
#define _STEPGEN_H

#include "rtapi.h"
#include "rtapi_app.h"
#include "hal.h"

#include "gpio.h"
#include "pulsgen.h"




#define STEPGEN_CH_CNT_MAX 16

typedef struct
{
    hal_bit_t *enable;

    hal_u32_t *step_space;
    hal_u32_t *step_len;
    hal_u32_t *dir_setup;
    hal_u32_t *dir_hold;

    hal_float_t *pos_scale;
    hal_float_t *maxvel;
    hal_float_t *maxaccel;

    hal_float_t *vel_cmd;
    hal_float_t *pos_cmd;

    hal_s32_t *rawcounts;
    hal_s32_t *counts;
    hal_float_t *freq;
    hal_float_t *pos_fb;
} stepgen_pin_t;

typedef struct
{
    uint8_t step_port;
    uint8_t step_pin;
    uint8_t dir_port;
    uint8_t dir_pin;

    uint8_t pulsgen_step_ch;
    uint8_t pulsgen_dir_ch;

    hal_float_t vel_cmd_old;
    hal_float_t pos_cmd_old;
} stepgen_data_t;








static int8_t *step_out;
RTAPI_MP_STRING(step_out, "Step output pins, comma separated");
static int8_t *dir_out;
RTAPI_MP_STRING(dir_out, "Direction output pins, comma separated");

static stepgen_pin_t *sg_pin;
static stepgen_data_t sg_dat[STEPGEN_CH_CNT_MAX] = {{0}};
static uint8_t stepgen_ch_cnt = 0;








static void stepgen_capture_pos(void *arg, long period)
{
    static uint8_t ch;
    for ( ch = stepgen_ch_cnt; ch--; )
    {
        if ( ! *sg_pin[ch].enable ) continue;
    }
}

static void stepgen_update_freq(void *arg, long period)
{
    static uint8_t ch;
    for ( ch = stepgen_ch_cnt; ch--; )
    {
        if ( ! *sg_pin[ch].enable ) continue;
    }
}




static int32_t stepgen_malloc_and_export(const char *comp_name, int32_t comp_id)
{
    int8_t* arg_str[2] = {step_out, dir_out};
    uint8_t n, ch;
    uint8_t pulsgen_ch = 0;
    int32_t r = 0;

    // find all output pins
    for ( n = 2; n--; )
    {
        if ( !arg_str[n] ) continue;

        int8_t *data = arg_str[n], *token;
        uint8_t port, pin, stepgen_ch = 0, found;
        int32_t retval;

        while ( (token = strtok(data, ",")) != NULL )
        {
            if ( data != NULL ) data = NULL;
            if ( stepgen_ch >= STEPGEN_CH_CNT_MAX ) continue;
            if ( strlen(token) < 3 ) continue;

            // trying to find a correct port name
            for ( found = 0, port = GPIO_PORTS_CNT; port--; )
            {
                if ( 0 == memcmp(token, gpio_name[port], 2) )
                {
                    found = 1;
                    break;
                }
            }

            if ( !found ) continue;

            // trying to find a correct pin number
            pin = (uint8_t) strtoul(&token[2], NULL, 10);

            if ( (pin == 0 && token[2] != '0') || pin >= GPIO_PINS_CNT ) continue;

            // save pin data
            if ( n )
            {
                sg_dat[stepgen_ch].dir_port = port;
                sg_dat[stepgen_ch].dir_pin = pin;
                sg_dat[stepgen_ch].pulsgen_dir_ch = pulsgen_ch;
            }
            else
            {
                sg_dat[stepgen_ch].step_port = port;
                sg_dat[stepgen_ch].step_pin = pin;
                sg_dat[stepgen_ch].pulsgen_step_ch = pulsgen_ch;
            }

            sg_dat[stepgen_ch].vel_cmd_old = 0.0;
            sg_dat[stepgen_ch].pos_cmd_old = 0.0;

            stepgen_ch++;
            pulsgen_ch++;
        }

        // update channels count
        if ( stepgen_ch > stepgen_ch_cnt ) stepgen_ch_cnt = stepgen_ch;
    }


    // shared memory allocation
    sg_pin = hal_malloc(stepgen_ch_cnt * sizeof(stepgen_pin_t *));
    if ( !sg_pin )
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: [STEPGEN] hal_malloc() failed \n", comp_name);
        return -1;
    }


    // export HAL pins
    #define EXPORT(TYPE,VAL,NAME,DEFAULT) \
        r += hal_pin_##TYPE##_newf(HAL_IO, &(sg_pin[ch].VAL), comp_id,\
        "%s.stepgen.%d." NAME, comp_name, ch);\
        *sg_pin[ch].VAL = DEFAULT;

    for ( r = 0, ch = stepgen_ch_cnt; ch--; )
    {
        EXPORT(bit,enable,"enable", 0);

        EXPORT(u32,step_space,"stepspace", 1);
        EXPORT(u32,step_len,"steplen", 1);
        EXPORT(u32,dir_setup,"dirsetup", 1);
        EXPORT(u32,dir_hold,"dirhold", 1);

        EXPORT(float,pos_scale,"position-scale", 1.0);
        EXPORT(float,maxvel,"maxvel", 0.0);
        EXPORT(float,maxaccel,"maxaccel", 0.0);

        EXPORT(float,vel_cmd,"velocity-cmd", 0.0);
        EXPORT(float,pos_cmd,"position-cmd", 0.0);

        EXPORT(s32,rawcounts,"rawcounts", 0);
        EXPORT(s32,counts,"counts", 0);
        EXPORT(float,freq,"frequency", 0.0);
        EXPORT(float,pos_fb,"position-fb", 0.0);
    }
    if ( r )
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: [STEPGEN] HAL pins export failed \n", comp_name);
        return -1;
    }

    #undef EXPORT


    // export HAL functions
    r = 0;
    r+= hal_export_functf(stepgen_capture_pos, 0, 1, 0, comp_id, "%s.stepgen.capture-position", comp_name);
    r+= hal_export_functf(stepgen_update_freq, 0, 1, 0, comp_id, "%s.stepgen.update-freq", comp_name);
    if ( r )
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: [STEPGEN] HAL functions export failed\n", comp_name);
        return -1;
    }

    return 0;
}




#endif
