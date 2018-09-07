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
#include <float.h>
#include "rtapi_math.h"

#include "gpio.h"
#include "pulsgen.h"




#define STEPGEN_CH_CNT_MAX 10

typedef struct
{
    // thess HAL pins must be public

    hal_bit_t *enable;

    hal_u64_t *step_space;
    hal_u64_t *step_len;
    hal_u64_t *dir_setup;
    hal_u64_t *dir_hold;

    hal_float_t *vel_max;
    hal_float_t *accel_max;

    hal_float_t *pos_scale;
    hal_float_t *pos_cmd;
    hal_float_t *pos_fb;

    // this HAL pins can be private

    hal_u64_t *step_port;
    hal_u64_t *step_pin;
    hal_bit_t *step_inv;
    hal_bit_t *step_state;
    hal_u64_t *step_pulsgen_ch0;
    hal_u64_t *step_pulsgen_ch1;
    hal_u64_t *step_task;
    hal_s64_t *step_pos;
    hal_s64_t *step_pos_target;
    hal_s64_t *step_task_dir0;
    hal_s64_t *step_task_dir1;
    hal_u64_t *step_task_t0;
    hal_u64_t *step_task_t1;
    hal_u64_t *step_task_t0_time;
    hal_u64_t *step_task_t1_time;
    hal_u64_t *step_freq;
    hal_u64_t *step_freq_old;
    hal_u64_t *step_freq_max;
    hal_u64_t *step_accel;
    hal_u64_t *step_accel_max;

    hal_u64_t *dir_port;
    hal_u64_t *dir_pin;
    hal_bit_t *dir_inv;
    hal_bit_t *dir_state;
    hal_u64_t *dir_pulsgen_ch;

    hal_bit_t *task;
    hal_u64_t *task_type;
    hal_u64_t *task_cnt;

    hal_float_t *pos_cmd_old;
} stepgen_pin_t;

typedef struct
{
    hal_u8_t step_port;
    hal_u8_t step_pin;
    hal_bit_t step_inv;
    hal_u8_t step_pulsgen_ch0;
    hal_u8_t step_pulsgen_ch1;

    hal_u8_t dir_port;
    hal_u8_t dir_pin;
    hal_bit_t dir_inv;
    hal_u8_t dir_pulsgen_ch;
} stepgen_data_t;

enum
{
    TASK_STEPS = 1,
    TASK_DIR,
    TASK_DIR_STEPS,
    TASK_STEPS_DIR,
    TASK_STEPS_DIR_STEPS
};









static int8_t *step_out;
RTAPI_MP_STRING(step_out, "Step output pins, comma separated");
static int8_t *dir_out;
RTAPI_MP_STRING(dir_out, "Direction output pins, comma separated");

static stepgen_pin_t *sg_pin;
static uint8_t stepgen_ch_cnt = 0;









#define sp sg_pin[ch]
#define s *sg_pin[ch]
#define u64v(PDBL) *((hal_u64_t*)PDBL)

static void stepgen_capture_pos(void *arg, long period)
{
    static hal_u8_t ch;
    static hal_u64_t t0, t1, d0;

    // check all used channels
    for ( ch = stepgen_ch_cnt; ch--; )
    {
        if ( !s.task ) goto pos_fb_calc;

        switch ( s.task_type )
        {
            case TASK_STEPS:
            {
                t0 = (hal_u64_t) pulsgen_task_toggles((hal_u32_t)s.step_pulsgen_ch0);
                s.step_pos += (s.dir_state ? -t0 : t0) / 2;
                break;
            }
        }

        pos_fb_calc:
        s.pos_fb = ((hal_float_t)s.step_pos) / s.pos_scale;
    }
}

static void stepgen_update_freq(void *arg, long period)
{
    static hal_u8_t ch;
    static hal_float_t pos_task;
    static hal_bit_t move_forward, dir_change, step_state, have_new_cmd;
    static hal_u64_t t0, toggle_time, steps_time, step_freq_max;
    static hal_s64_t step_pos, step_pos_target, step_task;

    for ( ch = stepgen_ch_cnt; ch--; )
    {
        // when channel is OFF
        if ( !s.enable )
        {
            s.step_freq = 0;
            s.step_freq_old = s.step_freq;
            s.pos_cmd_old = s.pos_cmd;

            // when channel is OFF and pulse generation is ON,
            // make a hard stop without deceleration (when ESTOP or something)
            if ( s.task )
            {
                switch ( s.task_type )
                {
                    case TASK_STEPS:
                    {
                        pulsgen_task_abort((hal_u32_t)s.step_pulsgen_ch0);
                        t0 = (hal_u64_t) pulsgen_task_toggles((hal_u32_t)s.step_pulsgen_ch0);

                        // complete the step pulse
                        if ( t0 && (t0 % 2) )
                        {
                            t0++;
                            if ( s.step_inv ) gpio_pin_set  (s.step_port, s.step_pin);
                            else              gpio_pin_clear(s.step_port, s.step_pin);
                        }

                        step_pos = s.step_pos + (s.dir_state ? -t0 : t0) / 2;

                        break;
                    }
                }

                s.step_pos = step_pos;
                s.task = 0;
            }

            continue;
        }


        // pre-process input data
        if ( !s.dir_hold ) s.dir_hold = 1000;
        if ( !s.dir_setup ) s.dir_setup = 1000;
        if ( !s.step_len ) s.step_len = 1000;
        if ( !s.step_space ) s.step_space = 1000;

        if ( s.pos_scale < 1e-20 && s.pos_scale > -1e-20 ) s.pos_scale = 1.0;

        s.step_freq_max = (hal_u64_t) rtapi_fabs((s.pos_scale) * (s.vel_max));
        if ( !s.step_freq_max ) s.step_freq_max = 999999999;
        step_freq_max = 1000000000 / (s.step_len + s.step_space);
        if ( s.step_freq_max > step_freq_max ) s.step_freq_max = step_freq_max;

        s.step_accel_max = (hal_u64_t) rtapi_fabs((s.pos_scale) * (s.accel_max));
        if ( !s.step_accel_max ) s.step_accel_max = 999999999;
        step_freq_max = s.step_freq_old + s.step_accel_max;
        if ( s.step_freq_max > step_freq_max ) s.step_freq_max = step_freq_max;


        // have we a new position command?
        have_new_cmd = (u64v(sp.pos_cmd) != u64v(sp.pos_cmd_old)) ? 1 : 0;
        step_pos_target = (hal_s64_t) (s.pos_scale * s.pos_cmd);
        have_new_cmd = (step_pos_target != s.step_pos) ? 1 : 0;



        /* now we have just a few ways to go:
            1.  if task is OFF and we have no position commands - do nothing;
            2.  if task is OFF and we have a new position commands - set new steps position and start task;
            3.  if task is ON and we have no position commands - change steps frequency;
            4.  if task is ON and we have a new position command - set new steps position and change steps frequency
        */



        if ( !s.task )
        {
            // 1
            if ( !have_new_cmd ) continue;

            // 2
            // get direction data
            move_forward = step_pos_target > s.step_pos ? 1 : 0;
            s.dir_state = gpio_pin_get(s.dir_port, s.dir_pin);
            if ( s.dir_inv ) s.dir_state = (hal_bit_t) ~(s.dir_state);
            dir_change = s.dir_state == move_forward ? 1 : 0;



        }
        else
        {
            // 3
            if ( !have_new_cmd )
            {

            }
            // 4
            else
            {

            }
        }


#if 0
        // when channel is ON and pulse generation is ON
        if ( s.task )
        {
            switch ( s.task_type )
            {
                case TASK_STEPS:
                {
                    t0 = (hal_u64_t) pulsgen_task_toggles((hal_u32_t)s.step_pulsgen_ch0);
                    step_pos = s.step_pos + (s.dir_state ? -t0 : t0) / 2;

                    // stop pulse generation when target position was achieved
                    if (
                        (s.dir_state && step_pos <= s.step_pos_target) ||
                        (!s.dir_state && step_pos >= s.step_pos_target)
                    ) {
                        pulsgen_task_abort((hal_u32_t)s.step_pulsgen_ch0);
                        t0 = (hal_u64_t) pulsgen_task_toggles((hal_u32_t)s.step_pulsgen_ch0);

                        // complete the step pulse
                        if ( t0 && (t0 % 2) )
                        {
                            t0++;
                            if ( s.step_inv ) gpio_pin_set  (s.step_port, s.step_pin);
                            else              gpio_pin_clear(s.step_port, s.step_pin);
                        }

                        step_pos = s.step_pos + (s.dir_state ? -t0 : t0) / 2;
                        s.step_freq = 0;
                        s.task = 0;
                    }

                    break;
                }
            }

            s.step_pos = step_pos;
        }
#endif

#if 0
        s.task_type = TASK_STEPS;
        s.step_task_dir0 = move_forward ? 1 : -1;

        toggle_time = steps_time / step_task / 2;
        s.step_task_t0_time = toggle_time;

        step_task *= 2;
        s.step_task = step_task;

        pulsgen_task_setup((hal_u32_t)s.step_pulsgen_ch0, (hal_u32_t)step_task,
            (hal_u32_t)toggle_time, (hal_u32_t)toggle_time, 0);
#endif
    }
}

#undef s
#undef sl
#undef u64v




static int32_t stepgen_malloc_and_export(const char *comp_name, int32_t comp_id)
{
    int8_t* arg_str[2] = {step_out, dir_out};
    uint8_t n, ch, pulsgen_ch = 0;
    int32_t r = 0;
    stepgen_data_t sg_dat[STEPGEN_CH_CNT_MAX] = {{0}};



    // find all output pins
    for ( n = 2; n--; )
    {
        if ( !arg_str[n] ) continue;

        int8_t *data = arg_str[n], *token, *inv_flag;
        uint8_t port, pin, inv = 0, stepgen_ch = 0, found;
        int32_t retval;

        while ( (token = strtok(data, ",")) != NULL )
        {
            if ( data != NULL ) data = NULL;
            if ( stepgen_ch >= STEPGEN_CH_CNT_MAX ) continue;
            if ( pulsgen_ch >= PULSGEN_CH_CNT ) continue;
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
            pin = (uint8_t) strtoul(&token[2], &inv_flag, 10);

            if ( (pin == 0 && token[2] != '0') || pin >= GPIO_PINS_CNT ) continue;

            // trying to find pin's invert flag
            if ( *inv_flag == '!' ) inv = 1;

            // save pin data
            if ( n ) // DIR pins
            {
                sg_dat[stepgen_ch].dir_port = port;
                sg_dat[stepgen_ch].dir_pin = pin;
                sg_dat[stepgen_ch].dir_inv = inv;
                sg_dat[stepgen_ch].dir_pulsgen_ch = pulsgen_ch;
                pulsgen_pin_setup(pulsgen_ch, port, pin, inv);
            }
            else // STEP pins
            {
                sg_dat[stepgen_ch].step_port = port;
                sg_dat[stepgen_ch].step_pin = pin;
                sg_dat[stepgen_ch].step_inv = inv;
                sg_dat[stepgen_ch].step_pulsgen_ch0 = pulsgen_ch;
                pulsgen_pin_setup(pulsgen_ch, port, pin, inv);
                sg_dat[stepgen_ch].step_pulsgen_ch1 = ++pulsgen_ch;
                pulsgen_pin_setup(pulsgen_ch, port, pin, inv);
            }

            stepgen_ch++;
            pulsgen_ch++;
        }

        // update channels count
        if ( stepgen_ch > stepgen_ch_cnt ) stepgen_ch_cnt = stepgen_ch;
    }


    // shared memory allocation
    sg_pin = hal_malloc(stepgen_ch_cnt * sizeof(stepgen_pin_t));
    if ( !sg_pin )
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: [STEPGEN] hal_malloc() failed \n", comp_name);
        return -1;
    }


#define EXPORT(IO_TYPE,TYPE,VAL,NAME,DEFAULT) \
    r += hal_pin_##TYPE##_newf(IO_TYPE, &(sg_pin[ch].VAL), comp_id,\
    "%s.stepgen.%d." NAME, comp_name, ch);\
    *sg_pin[ch].VAL = DEFAULT;

    // export HAL pins
    for ( r = 0, ch = stepgen_ch_cnt; ch--; )
    {
        EXPORT(HAL_IN,bit,enable,"enable", 0);
        EXPORT(HAL_IN,u64,step_space,"step_space", 1);
        EXPORT(HAL_IN,u64,step_len,"step_len", 1);
        EXPORT(HAL_IN,u64,dir_setup,"dir_setup", 1);
        EXPORT(HAL_IN,u64,dir_hold,"dir_hold", 1);
        EXPORT(HAL_IN,float,pos_scale,"pos_scale", 1.0);
        EXPORT(HAL_IN,float,vel_max,"vel_max", 0.0);
        EXPORT(HAL_IN,float,accel_max,"accel_max", 0.0);
        EXPORT(HAL_IN,float,pos_cmd,"pos_cmd", 0.0);
        EXPORT(HAL_OUT,float,pos_fb,"pos_fb", 0.0);

        EXPORT(HAL_IN,u64,step_port,"step_port", sg_dat[ch].step_port);
        EXPORT(HAL_IN,u64,step_pin,"step_pin", sg_dat[ch].step_pin);
        EXPORT(HAL_IN,bit,step_inv,"step_inv", sg_dat[ch].step_inv);
        EXPORT(HAL_OUT,bit,step_state,"step_state", 0);
        EXPORT(HAL_OUT,u64,step_pulsgen_ch0,"step_pulsgen_ch0", sg_dat[ch].step_pulsgen_ch0);
        EXPORT(HAL_OUT,u64,step_pulsgen_ch1,"step_pulsgen_ch1", sg_dat[ch].step_pulsgen_ch1);
        EXPORT(HAL_OUT,u64,step_task,"step_task", 0);
        EXPORT(HAL_OUT,s64,step_pos,"step_pos", 0);
        EXPORT(HAL_OUT,s64,step_pos_target,"step_pos_target", 0);
        EXPORT(HAL_OUT,s64,step_task_dir0,"step_task_dir0", 0);
        EXPORT(HAL_OUT,s64,step_task_dir1,"step_task_dir1", 0);
        EXPORT(HAL_OUT,u64,step_task_t0,"step_task_t0", 0);
        EXPORT(HAL_OUT,u64,step_task_t1,"step_task_t1", 0);
        EXPORT(HAL_OUT,u64,step_task_t0_time,"step_task_t0_time", 0);
        EXPORT(HAL_OUT,u64,step_task_t1_time,"step_task_t1_time", 0);
        EXPORT(HAL_OUT,u64,step_freq,"step_freq", 0);
        EXPORT(HAL_OUT,u64,step_freq_old,"step_freq_old", 0);
        EXPORT(HAL_OUT,u64,step_freq_max,"step_freq_max", 0);
        EXPORT(HAL_OUT,u64,step_accel,"step_accel", 0);
        EXPORT(HAL_OUT,u64,step_accel_max,"step_accel_max", 0);
        EXPORT(HAL_IN,u64,dir_port,"dir_port", sg_dat[ch].dir_port);
        EXPORT(HAL_IN,u64,dir_pin,"dir_pin", sg_dat[ch].dir_pin);
        EXPORT(HAL_IN,bit,dir_inv,"dir_inv", sg_dat[ch].dir_inv);
        EXPORT(HAL_OUT,bit,dir_state,"dir_state", 0);
        EXPORT(HAL_OUT,u64,dir_pulsgen_ch,"dir_pulsgen_ch", sg_dat[ch].dir_pulsgen_ch);
        EXPORT(HAL_OUT,bit,task,"task", 0);
        EXPORT(HAL_OUT,u64,task_type,"task_type", 0);
        EXPORT(HAL_OUT,u64,task_cnt,"task_cnt", 0);
        EXPORT(HAL_OUT,float,pos_cmd_old,"pos_cmd_old", 0.0);
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
