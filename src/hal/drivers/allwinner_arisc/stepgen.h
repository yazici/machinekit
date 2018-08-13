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

    hal_u32_t *step_space;
    hal_u32_t *step_len;
    hal_u32_t *dir_setup;
    hal_u32_t *dir_hold;

    hal_float_t *vel_max;
    hal_float_t *accel_max;

    hal_float_t *pos_scale;
    hal_float_t *pos_cmd;
    hal_float_t *pos_fb;

    // this HAL pins can be private

    hal_u32_t *step_port;
    hal_u32_t *step_pin;
    hal_bit_t *step_inv;
    hal_bit_t *step_state;
    hal_u32_t *step_pulsgen_ch0;
    hal_u32_t *step_pulsgen_ch1;
    hal_s32_t *step_task_dir0;
    hal_s32_t *step_task_dir1;
    hal_u32_t *step_task_toggles0;
    hal_u32_t *step_task_toggles1;
    hal_u64_t *step_freq;
    hal_u64_t *step_freq_old;
    hal_u64_t *step_freq_max;
    hal_u64_t *step_accel;
    hal_u64_t *step_accel_max;

    hal_u32_t *dir_port;
    hal_u32_t *dir_pin;
    hal_bit_t *dir_inv;
    hal_bit_t *dir_state;
    hal_u32_t *dir_pulsgen_ch;

    hal_bit_t *task;
    hal_u32_t *task_type;

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
    TASK_STEPS,
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








static void stepgen_capture_pos(void *arg, long period)
{
    static uint8_t ch, pos_changed;
    static hal_bit_t dir_state;
    static uint32_t t0, t1, d0;

    // check all used channels
    for ( ch = stepgen_ch_cnt; ch--; )
    {
        // keep saving some data if channel is disabled or idle
        if ( ! *sg_pin[ch].enable || ! *sg_pin[ch].task )
        {
            *sg_pin[ch].step_freq = 0;
            *sg_pin[ch].pos_cmd_old = *sg_pin[ch].pos_cmd;
            continue;
        }

        pos_changed = 0;

        // what kind of task it was?
        switch ( *sg_pin[ch].task_type )
        {
            case TASK_STEPS:
            {
                // abort last pulsgen tasks and get pin toggles count
                pulsgen_task_abort(*sg_pin[ch].step_pulsgen_ch0);
                t0 = pulsgen_task_toggles(*sg_pin[ch].step_pulsgen_ch0);

                // update pin states
                *sg_pin[ch].step_state = t0 && (t0 % 2) ? 1 : 0;

                // if needs make a half step to complete the task
                if ( *sg_pin[ch].step_state )
                {
                    if ( *sg_pin[ch].step_inv ) {
                        gpio_pin_set(*sg_pin[ch].step_port, *sg_pin[ch].step_pin);
                    } else {
                        gpio_pin_clear(*sg_pin[ch].step_port, *sg_pin[ch].step_pin);
                    }

                    t0++;
                    *sg_pin[ch].step_state = 0;
                }

                // position is same
                if ( !t0 ) break;

                // update HAL feedback pin
                *sg_pin[ch].pos_fb = *sg_pin[ch].pos_cmd +
                    ((hal_float_t)(t0/2 * (*sg_pin[ch].step_task_dir0))) *
                    (*sg_pin[ch].pos_scale);

                // position was changed
                pos_changed = 1;
                break;
            }

            case TASK_DIR:
            {
                pulsgen_task_abort(*sg_pin[ch].dir_pulsgen_ch);
                d0 = pulsgen_task_toggles(*sg_pin[ch].dir_pulsgen_ch);

                dir_state = d0 && (d0 % 2) ? 1 : *sg_pin[ch].dir_state;

                if ( dir_state == *sg_pin[ch].dir_state )
                {
                    if ( *sg_pin[ch].dir_inv ^ dir_state ) {
                        gpio_pin_set(*sg_pin[ch].dir_port, *sg_pin[ch].dir_pin);
                    } else {
                        gpio_pin_clear(*sg_pin[ch].dir_port, *sg_pin[ch].dir_pin);
                    }
                }

                *sg_pin[ch].dir_state = *sg_pin[ch].dir_state ? 0 : 1;

                break;
            }

            case TASK_DIR_STEPS:
            {
                pulsgen_task_abort(*sg_pin[ch].dir_pulsgen_ch);
                pulsgen_task_abort(*sg_pin[ch].step_pulsgen_ch0);
                d0 = pulsgen_task_toggles(*sg_pin[ch].dir_pulsgen_ch);
                t0 = pulsgen_task_toggles(*sg_pin[ch].step_pulsgen_ch0);

                dir_state = d0 && (d0 % 2) ? 1 : *sg_pin[ch].dir_state;

                if ( dir_state == *sg_pin[ch].dir_state )
                {
                    if ( *sg_pin[ch].dir_inv ^ dir_state ) {
                        gpio_pin_set(*sg_pin[ch].dir_port, *sg_pin[ch].dir_pin);
                    } else {
                        gpio_pin_clear(*sg_pin[ch].dir_port, *sg_pin[ch].dir_pin);
                    }
                }

                *sg_pin[ch].dir_state = *sg_pin[ch].dir_state ? 0 : 1;
                *sg_pin[ch].step_state = t0 && (t0 % 2) ? 1 : 0;

                if ( *sg_pin[ch].step_state )
                {
                    if ( *sg_pin[ch].step_inv ) {
                        gpio_pin_set(*sg_pin[ch].step_port, *sg_pin[ch].step_pin);
                    } else {
                        gpio_pin_clear(*sg_pin[ch].step_port, *sg_pin[ch].step_pin);
                    }

                    t0++;
                    *sg_pin[ch].step_state = 0;
                }

                if ( !t0 ) break;

                *sg_pin[ch].pos_fb = *sg_pin[ch].pos_cmd +
                    (dir_state ? -1.0 : 1.0) *
                    ((hal_float_t)(t0/2 * (*sg_pin[ch].step_task_dir0))) *
                    (*sg_pin[ch].pos_scale);

                pos_changed = 1;
                break;
            }

            case TASK_STEPS_DIR:
            {
                pulsgen_task_abort(*sg_pin[ch].step_pulsgen_ch0);
                pulsgen_task_abort(*sg_pin[ch].dir_pulsgen_ch);
                d0 = pulsgen_task_toggles(*sg_pin[ch].dir_pulsgen_ch);
                t0 = pulsgen_task_toggles(*sg_pin[ch].step_pulsgen_ch0);

                dir_state = d0 && (d0 % 2) ? 1 : *sg_pin[ch].dir_state;

                if ( dir_state == *sg_pin[ch].dir_state )
                {
                    if ( *sg_pin[ch].dir_inv ^ dir_state ) {
                        gpio_pin_set(*sg_pin[ch].dir_port, *sg_pin[ch].dir_pin);
                    } else {
                        gpio_pin_clear(*sg_pin[ch].dir_port, *sg_pin[ch].dir_pin);
                    }
                }

                *sg_pin[ch].dir_state = *sg_pin[ch].dir_state ? 0 : 1;
                *sg_pin[ch].step_state = t0 && (t0 % 2) ? 1 : 0;

                if ( *sg_pin[ch].step_state )
                {
                    if ( *sg_pin[ch].step_inv ) {
                        gpio_pin_set(*sg_pin[ch].step_port, *sg_pin[ch].step_pin);
                    } else {
                        gpio_pin_clear(*sg_pin[ch].step_port, *sg_pin[ch].step_pin);
                    }

                    t0++;
                    *sg_pin[ch].step_state = 0;
                }

                if ( !t0 ) break;

                *sg_pin[ch].pos_fb = *sg_pin[ch].pos_cmd +
                    ((hal_float_t)(t0/2 * (*sg_pin[ch].step_task_dir0))) *
                    (*sg_pin[ch].pos_scale);

                pos_changed = 1;
                break;
            }

            case TASK_STEPS_DIR_STEPS:
            {
                pulsgen_task_abort(*sg_pin[ch].step_pulsgen_ch0);
                pulsgen_task_abort(*sg_pin[ch].dir_pulsgen_ch);
                pulsgen_task_abort(*sg_pin[ch].step_pulsgen_ch1);
                t0 = pulsgen_task_toggles(*sg_pin[ch].step_pulsgen_ch0);
                d0 = pulsgen_task_toggles(*sg_pin[ch].dir_pulsgen_ch);
                t1 = pulsgen_task_toggles(*sg_pin[ch].step_pulsgen_ch1);

                dir_state = d0 && (d0 % 2) ? 1 : *sg_pin[ch].dir_state;

                if ( dir_state == *sg_pin[ch].dir_state )
                {
                    if ( *sg_pin[ch].dir_inv ^ dir_state ) {
                        gpio_pin_set(*sg_pin[ch].dir_port, *sg_pin[ch].dir_pin);
                    } else {
                        gpio_pin_clear(*sg_pin[ch].dir_port, *sg_pin[ch].dir_pin);
                    }
                }

                *sg_pin[ch].dir_state = *sg_pin[ch].dir_state ? 0 : 1;
                *sg_pin[ch].step_state = (t0+t1) && ((t0+t1) % 2) ? 1 : 0;

                if ( *sg_pin[ch].step_state )
                {
                    if ( *sg_pin[ch].step_inv ) {
                        gpio_pin_set(*sg_pin[ch].step_port, *sg_pin[ch].step_pin);
                    } else {
                        gpio_pin_clear(*sg_pin[ch].step_port, *sg_pin[ch].step_pin);
                    }

                    t1++;
                    *sg_pin[ch].step_state = 0;
                }

                if ( !t0 && !t1 ) break;

                *sg_pin[ch].pos_fb = *sg_pin[ch].pos_cmd +
                    ((hal_float_t)(t0/2 * (*sg_pin[ch].step_task_dir0))) *
                    (*sg_pin[ch].pos_scale);

                *sg_pin[ch].pos_fb +=
                    (dir_state ? -1.0 : 1.0) *
                    ((hal_float_t)(t1/2 * (*sg_pin[ch].step_task_dir1))) *
                    (*sg_pin[ch].pos_scale);

                pos_changed = 1;
                break;
            }
        }

        if ( !pos_changed ) *sg_pin[ch].pos_fb = *sg_pin[ch].pos_cmd;

        // save current position
        *sg_pin[ch].pos_cmd_old = *sg_pin[ch].pos_cmd;

        // task done
        *sg_pin[ch].task = 0;
    }
}

static void stepgen_update_freq(void *arg, long period)
{
    static hal_u8_t ch;
    static hal_bit_t move_forward, dir_change;
    static hal_float_t pos_task;
    static hal_s32_t step_task;

#define sp sg_pin[ch]
#define s *sg_pin[ch]
#define u64v(PDBL) *((hal_u64_t*)PDBL)

    for ( ch = stepgen_ch_cnt; ch--; )
    {
        // goto next channel if current is off or idle
        if ( !s.enable ) continue;
        if ( u64v(sp.pos_cmd) == u64v(sp.pos_cmd_old) ) continue;

        // get task data
        pos_task = s.pos_cmd - s.pos_cmd_old;
        step_task = (hal_s32_t) (s.pos_scale * rtapi_fabs(pos_task));
        move_forward = pos_task > 0.0 ? 1 : 0;
        dir_change = s.dir_state == move_forward ? 1 : 0;

        // get frequency and acceleration limits
        s.step_freq_max = (hal_u64_t) rtapi_fabs((s.pos_scale) * (s.vel_max));
        s.step_accel_max = (hal_u64_t) rtapi_fabs((s.pos_scale) * (s.accel_max));
        if ( !s.step_freq_max ) s.step_freq_max = UINT64_MAX;
        if ( !s.step_accel_max ) s.step_accel_max = UINT64_MAX;

        // get current steps frequency
        s.step_freq_old = s.step_freq;
        s.step_freq = step_task * 1000000000 / period;
        if ( s.step_freq > s.step_freq_max )
        {
            step_task = step_task * s.step_freq_max / s.step_freq;
            s.step_freq = step_task * 1000000000 / period;
        }

        // get current steps acceleration
        s.step_accel = s.step_freq_old > s.step_freq ?
            s.step_freq_old - s.step_freq :
            s.step_freq - s.step_freq_old ;
        if ( s.step_accel > s.step_accel_max )
        {
            step_task = step_task * s.step_accel_max / s.step_accel;
            s.step_freq = step_task * 1000000000 / period;
        }

        // goto next channel if nothing to do here
        if ( !step_task && !dir_change ) continue;

        // we have a task
        s.task = 1;

        // set task for the pulsgen
        if ( dir_change ) // with DIR change
        {
            if ( !step_task )
            {
                s.task_type = TASK_DIR;
                pulsgen_task_setup(s.dir_pulsgen_ch, 1, period/2, period/2, 0);
                continue;
            }

            // TODO
        }
        else // just a few steps
        {
            s.task_type = TASK_STEPS;
            s.step_task_dir0 = move_forward ? 1 : -1;

            // TODO - pin setup time and pin hold time calculation
            // based on step_setup/step_hold

            pulsgen_task_setup(
                s.step_pulsgen_ch0,
                step_task*2*100/80, // add 20% to the step_task value
                period/step_task/2, // pin setup time
                period/step_task/2, // pin hold time
                0 // no startup delay
            );
        }
    }

#undef s
#undef sl
#undef u64v
}




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


    // export HAL pins
    #define EXPORT(TYPE,VAL,NAME,DEFAULT) \
        r += hal_pin_##TYPE##_newf(HAL_IO, &(sg_pin[ch].VAL), comp_id,\
        "%s.stepgen.%d." NAME, comp_name, ch);\
        *sg_pin[ch].VAL = DEFAULT;

    for ( r = 0, ch = stepgen_ch_cnt; ch--; )
    {
        EXPORT(bit,enable,"enable", 0);
        EXPORT(u32,step_space,"step_space", 1);
        EXPORT(u32,step_len,"step_len", 1);
        EXPORT(u32,dir_setup,"dir_setup", 1);
        EXPORT(u32,dir_hold,"dir_hold", 1);
        EXPORT(float,pos_scale,"pos_scale", 1.0);
        EXPORT(float,vel_max,"vel_max", 0.0);
        EXPORT(float,accel_max,"accel_max", 0.0);
        EXPORT(float,pos_cmd,"pos_cmd", 0.0);
        EXPORT(float,pos_fb,"pos_fb", 0.0);

        EXPORT(u32,step_port,"step_port", sg_dat[ch].step_port);
        EXPORT(u32,step_pin,"step_pin", sg_dat[ch].step_pin);
        EXPORT(bit,step_inv,"step_inv", sg_dat[ch].step_inv);
        EXPORT(bit,step_state,"step_state", 0);
        EXPORT(u32,step_pulsgen_ch0,"step_pulsgen_ch0", sg_dat[ch].step_pulsgen_ch0);
        EXPORT(u32,step_pulsgen_ch1,"step_pulsgen_ch1", sg_dat[ch].step_pulsgen_ch1);
        EXPORT(s32,step_task_dir0,"step_task_dir0", 0);
        EXPORT(s32,step_task_dir1,"step_task_dir1", 0);
        EXPORT(u32,step_task_toggles0,"step_task_toggles0", 0);
        EXPORT(u32,step_task_toggles1,"step_task_toggles1", 0);
        EXPORT(u64,step_freq,"step_freq", 0);
        EXPORT(u64,step_freq_old,"step_freq_old", 0);
        EXPORT(u64,step_freq_max,"step_freq_max", 0);
        EXPORT(u64,step_accel,"step_accel", 0);
        EXPORT(u64,step_accel_max,"step_accel_max", 0);
        EXPORT(u32,dir_port,"dir_port", sg_dat[ch].dir_port);
        EXPORT(u32,dir_pin,"dir_pin", sg_dat[ch].dir_pin);
        EXPORT(bit,dir_inv,"dir_inv", sg_dat[ch].dir_inv);
        EXPORT(bit,dir_state,"dir_state", 0);
        EXPORT(u32,dir_pulsgen_ch,"dir_pulsgen_ch", sg_dat[ch].dir_pulsgen_ch);
        EXPORT(bit,task,"task", 0);
        EXPORT(u32,task_type,"task_type", 0);
        EXPORT(float,pos_cmd_old,"pos_cmd_old", 0.0);
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
