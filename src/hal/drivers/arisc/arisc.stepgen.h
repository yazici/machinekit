#ifndef _ARISC_STEPGEN_H
#define _ARISC_STEPGEN_H

#include "hal.h"




#define STEPGEN_CH_CNT_MAX      16
#define STEPGEN_DIR_TIME_MAX    200000 // max(dirhold + dirsetup), ns

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

    hal_bit_t *enable; // in

    hal_u32_t *step_space; // in
    hal_u32_t *step_len; // in
    hal_u32_t *step_port; // in
    hal_u32_t *step_pin; // in
    hal_bit_t *step_inv; // in

    hal_u32_t *dir_setup; // in
    hal_u32_t *dir_hold; // in
    hal_u32_t *dir_port; // in
    hal_u32_t *dir_pin; // in
    hal_bit_t *dir_inv; // in

    hal_float_t *vel_max; // in
    hal_float_t *accel_max; // in
    hal_float_t *pos_scale; // in
    hal_float_t *pos_cmd; // in
    hal_float_t *vel_cmd; // in

    hal_s32_t *counts; // out
    hal_s32_t *rawcounts; // out
    hal_float_t *pos_fb; // out
    hal_float_t *freq; // out

    hal_s32_t counts_new; // private
    hal_bit_t ctrl_type; // private

    hal_u32_t step_space_old; // private
    hal_u32_t step_len_old; // private
    hal_u32_t step_port_old; // private
    hal_u32_t step_pin_old; // private
    hal_bit_t step_inv_old; // private
    hal_s32_t step_freq_new; // private
    hal_s32_t step_freq; // private
    hal_u32_t step_freq_max; // private
    hal_u32_t step_accel_max; // private

    hal_u32_t dir_port_old; // private
    hal_u32_t dir_pin_old; // private
    hal_bit_t dir_inv_old; // private

    hal_float_t vel_max_old; // private
    hal_float_t accel_max_old; // private
    hal_float_t pos_cmd_old; // private
    hal_float_t vel_cmd_old; // private

} stepgen_ch_t;




#endif
