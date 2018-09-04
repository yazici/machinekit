/********************************************************************
* Description: relkins.c
*
*   Trivial kinematics for 3 axis Cartesian machine
*   with position adjustment
*
* Author: Mikhail Vydrenko (mikhail@vydrenko.ru)
********************************************************************/

#define FOR_MACHINEKIT 1

#include "kinematics.h"     /* these decls */
#include "rtapi.h"          /* RTAPI realtime OS API */
#include "rtapi_app.h"      /* RTAPI realtime module decls */
#include "hal.h"

#if FOR_MACHINEKIT
#include <stdlib.h>
#else
#include "motion.h"
#include "rtapi_math.h"
#include "rtapi_string.h"
#include <linux/kernel.h>
#endif




#define RELKINS_FWD 1
#define RELKINS_REV 1
#define AXIS_CNT_MAX 9
#define AXIS_STEPS_MAX 255

#if FOR_MACHINEKIT
#define VTVERSION VTKINEMATICS_VERSION1
#endif

MODULE_AUTHOR("Mikhail Vydrenko");
MODULE_DESCRIPTION("Trivial kinematics with position adjustment");
MODULE_LICENSE("GPL");

#if FOR_MACHINEKIT
static int vtable_id;
#endif
static int comp_id;
static const char *name = "relkins";
static const char *axis_name = "XYZABCUVW";

static hal_bit_t axis_adj[AXIS_CNT_MAX] = {0};

hal_float_t **  _adj[AXIS_CNT_MAX];
hal_u32_t **    _steps;
hal_float_t **  _step_size;
hal_float_t **  _start_from;
hal_u32_t **    _rel_axis;

static char *steps;
RTAPI_MP_STRING(steps, "steps count, comma separated");
static char *step_size;
RTAPI_MP_STRING(step_size, "step size in units, comma separated");
static char *start_from;
RTAPI_MP_STRING(start_from, "start position offset in units, comma separated");
static char *rel_axis;
RTAPI_MP_STRING(rel_axis, "related axis name/ID, comma separated");




int
kinematicsForward
(
    const double *                      joints,
    EmcPose *                           pos,
    const KINEMATICS_FORWARD_FLAGS *    fflags,
    KINEMATICS_INVERSE_FLAGS *          iflags
)
{
#if RELKINS_FWD
    static hal_u32_t step_0, step_1, axis;
    static hal_float_t adj, adj_0, adj_1;
    hal_float_t *axes[AXIS_CNT_MAX] =
    {
        &(pos->tran.x), &(pos->tran.y), &(pos->tran.z),
        &(pos->a), &(pos->b), &(pos->c),
        &(pos->u), &(pos->v), &(pos->w)
    };

    for ( axis = AXIS_CNT_MAX; axis--; )
    {
        if ( !axis_adj[axis] || joints[*_rel_axis[axis]] < 1e-20 )
        {
            *axes[axis] = joints[axis];
            continue;
        }

        if ( *_step_size[axis] < 1e-20 ) *_step_size[axis] = 1.0;

        step_0 = (hal_u32_t) ( joints[*_rel_axis[axis]] / (*_step_size[axis]) );
        step_1 = step_0 + 1;

        adj_0 = step_0 >= 0 && step_0 < *_steps[axis] ? *_adj[axis][step_0] : 0.0;
        adj_1 = step_1 >= 0 && step_1 < *_steps[axis] ? *_adj[axis][step_1] : 0.0;

        adj =   adj_0
              + (adj_1 - adj_0)
              * (joints[*_rel_axis[axis]] - (*_step_size[axis]) * ((hal_float_t)step_0));

        *axes[axis] = joints[axis] + adj;
    }
#else
    pos->tran.x = joints[0];
    pos->tran.y = joints[1];
    pos->tran.z = joints[2];
    pos->a = joints[3];
    pos->b = joints[4];
    pos->c = joints[5];
    pos->u = joints[6];
    pos->v = joints[7];
    pos->w = joints[8];
#endif
    return 0;
}

int
kinematicsInverse
(
    const EmcPose *                     pos,
    double *                            joints,
    const KINEMATICS_INVERSE_FLAGS *    iflags,
    KINEMATICS_FORWARD_FLAGS *          fflags
)
{
#if RELKINS_REV
    static hal_u32_t step_0, step_1, axis;
    static hal_float_t adj, adj_0, adj_1;
    const hal_float_t *axes[AXIS_CNT_MAX] =
    {
        &(pos->tran.x), &(pos->tran.y), &(pos->tran.z),
        &(pos->a), &(pos->b), &(pos->c),
        &(pos->u), &(pos->v), &(pos->w)
    };

    for ( axis = AXIS_CNT_MAX; axis--; )
    {
        if ( !axis_adj[axis] || *axes[*_rel_axis[axis]] < 1e-20 )
        {
            joints[axis] = *axes[axis];
            continue;
        }

        if ( *_step_size[axis] < 1e-20 ) *_step_size[axis] = 1.0;

        step_0 = (hal_u32_t) ( *axes[*_rel_axis[axis]] / (*_step_size[axis]) );
        step_1 = step_0 + 1;

        adj_0 = step_0 >= 0 && step_0 < *_steps[axis] ? *_adj[axis][step_0] : 0.0;
        adj_1 = step_1 >= 0 && step_1 < *_steps[axis] ? *_adj[axis][step_1] : 0.0;

        adj =   adj_0
              + (adj_1 - adj_0)
              * (*axes[*_rel_axis[axis]] - (*_step_size[axis]) * ((hal_float_t)step_0));

        joints[axis] = *axes[axis] - adj;
    }
#else
    joints[0] = pos->tran.x;
    joints[1] = pos->tran.y;
    joints[2] = pos->tran.z;
    joints[3] = pos->a;
    joints[4] = pos->b;
    joints[5] = pos->c;
    joints[6] = pos->u;
    joints[7] = pos->v;
    joints[8] = pos->w;
#endif
    return 0;
}

/* implemented for these kinematics as giving joints preference */
int
kinematicsHome
(
    EmcPose *                   world,
    double *                    joint,
    KINEMATICS_FORWARD_FLAGS *  fflags,
    KINEMATICS_INVERSE_FLAGS *  iflags
)
{
    *fflags = 0;
    *iflags = 0;

    return kinematicsForward(joint, world, fflags, iflags);
}

KINEMATICS_TYPE
kinematicsType(void)
{
    return KINEMATICS_IDENTITY;
}

#if FOR_MACHINEKIT
static vtkins_t vtk =
{
    .kinematicsForward = kinematicsForward,
    .kinematicsInverse  = kinematicsInverse,
//  .kinematicsHome = kinematicsHome,
    .kinematicsType = kinematicsType
};
#else
EXPORT_SYMBOL(kinematicsType);
EXPORT_SYMBOL(kinematicsForward);
EXPORT_SYMBOL(kinematicsInverse);
#endif




int rtapi_app_main(void)
{
    hal_s32_t retval;
#if !FOR_MACHINEKIT
    int32_t n;
#endif
    uint32_t axis, step, steps_cnt[AXIS_CNT_MAX] = {0};
    char *data, *token;

    // component init
    comp_id = hal_init(name);
    if ( comp_id < 0 ) return -1;

#if FOR_MACHINEKIT
    // export kinematics table
    vtable_id = hal_export_vtable(name, VTVERSION, &vtk, comp_id);
    if ( vtable_id < 0 )
    {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "%s: ERROR: hal_export_vtable(%s,%d,%p) failed: %d\n",
            name, name, VTVERSION, &vtk, vtable_id);
        return -1;
    }
#endif

    // shared memory allocation
    _steps      = hal_malloc(AXIS_CNT_MAX * sizeof(hal_u32_t*));
    _step_size  = hal_malloc(AXIS_CNT_MAX * sizeof(hal_float_t*));
    _start_from = hal_malloc(AXIS_CNT_MAX * sizeof(hal_float_t*));
    _rel_axis   = hal_malloc(AXIS_CNT_MAX * sizeof(hal_u32_t*));
    if ( !_steps || !_step_size || !_rel_axis || !_start_from )
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: info pins hal_malloc() failed \n", name);
    }

    // parse parameter string
    if ( steps != NULL )
    {
#if FOR_MACHINEKIT
        for ( axis = 0, data = steps; (token = strtok(data, ",")) != NULL; axis++ )
        {
            if ( data != NULL ) data = NULL;
#else
        for ( axis = 0, data = steps; (token = strsep(&data, ",")) != NULL; axis++ )
        {
#endif

            // get steps count
#if FOR_MACHINEKIT
            steps_cnt[axis] = (hal_u32_t) strtoul(token, NULL, 10);
#else
            retval = kstrtou32(token, 10, &steps_cnt[axis]);
#endif
            if ( steps_cnt[axis] <= 0 ) continue;
            if ( steps_cnt[axis] > AXIS_STEPS_MAX ) steps_cnt[axis] = AXIS_STEPS_MAX;

            // shared memory allocation
            _adj[axis] = hal_malloc(steps_cnt[axis] * sizeof(hal_float_t*));
            if ( !_adj[axis] )
            {
                rtapi_print_msg(RTAPI_MSG_ERR, "%s: step pins hal_malloc() failed \n", name);
                return -1;
            }

            // export step pins
            for ( step = 0, retval = 0; step < steps_cnt[axis]; step++ )
            {
                retval += hal_pin_float_newf(HAL_IN, &_adj[axis][step],
                              comp_id, "%s.%c.%d", name, axis_name[axis], step);
            }
            if ( retval )
            {
                rtapi_print_msg(RTAPI_MSG_ERR, "%s: HAL step pins export failed \n", name);
                return -1;
            }
        }
    }

    // export info pins
    for ( axis = 0, retval = 0; axis < AXIS_CNT_MAX; axis++ )
    {
        axis_adj[axis] = steps_cnt[axis] ? 1 : 0;
        if ( !axis_adj[axis] ) continue;

        retval += hal_pin_u32_newf  (HAL_OUT, &_steps[axis],      comp_id, "%s.%c.steps",      name, axis_name[axis]);
        retval += hal_pin_float_newf(HAL_IN,  &_step_size[axis],  comp_id, "%s.%c.step_size",  name, axis_name[axis]);
        retval += hal_pin_float_newf(HAL_IN,  &_start_from[axis], comp_id, "%s.%c.start_from", name, axis_name[axis]);
        retval += hal_pin_u32_newf  (HAL_IN,  &_rel_axis[axis],   comp_id, "%s.%c.rel_axis",   name, axis_name[axis]);

        *_steps[axis] = steps_cnt[axis];
    }
    if ( retval )
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: HAL info pins export failed \n", name);
        return -1;
    }

    // parse parameter string
    if ( step_size != NULL )
    {
#if FOR_MACHINEKIT
        for ( axis = 0, data = step_size; (token = strtok(data, ",")) != NULL; axis++ )
        {
            if ( data != NULL ) data = NULL;
#else
        for ( axis = 0, data = step_size; (token = strsep(&data, ",")) != NULL; axis++ )
        {
#endif
            if ( !axis_adj[axis] ) continue;

#if FOR_MACHINEKIT
            *_step_size[axis] = (hal_float_t) strtod(token, NULL);
#else
            retval = kstrtou32(token, 10, &n);
            *_step_size[axis] = (hal_float_t) n;
#endif
        }
    }

    // parse parameter string
    if ( start_from != NULL )
    {
#if FOR_MACHINEKIT
        for ( axis = 0, data = start_from; (token = strtok(data, ",")) != NULL; axis++ )
        {
            if ( data != NULL ) data = NULL;
#else
        for ( axis = 0, data = start_from; (token = strsep(&data, ",")) != NULL; axis++ )
        {
#endif
            if ( !axis_adj[axis] ) continue;

#if FOR_MACHINEKIT
            *_start_from[axis] = (hal_float_t) strtod(token, NULL);
#else
            retval = kstrtou32(token, 10, &n);
            *_start_from[axis] = (hal_float_t) n;
#endif
        }
    }

    // parse parameter string
    if ( rel_axis != NULL )
    {
#if FOR_MACHINEKIT
        for ( axis = 0, data = rel_axis; (token = strtok(data, ",")) != NULL; axis++ )
        {
            if ( data != NULL ) data = NULL;
#else
        for ( axis = 0, data = rel_axis; (token = strsep(&data, ",")) != NULL; axis++ )
        {
#endif
            if ( !axis_adj[axis] ) continue;

            // parse axis id as number
            if ( token[0] >= '0' && token[0] <= '9' )
            {
#if FOR_MACHINEKIT
                *_rel_axis[axis] = (hal_u32_t) strtol(token, NULL, 10);
#else
                retval = kstrtou32(token, 10, (uint32_t*)_rel_axis[axis]);
#endif
                if ( *_rel_axis[axis] >= AXIS_CNT_MAX ) *_rel_axis[axis] = AXIS_CNT_MAX - 1;
            }
            // parse axis id as char
            else
            {
                switch ( token[0] )
                {
                    case 'x': case 'X': *_rel_axis[axis] = 0; break;
                    case 'y': case 'Y': *_rel_axis[axis] = 1; break;
                    case 'z': case 'Z': *_rel_axis[axis] = 2; break;
                    case 'a': case 'A': *_rel_axis[axis] = 3; break;
                    case 'b': case 'B': *_rel_axis[axis] = 4; break;
                    case 'c': case 'C': *_rel_axis[axis] = 5; break;
                    case 'u': case 'U': *_rel_axis[axis] = 6; break;
                    case 'v': case 'V': *_rel_axis[axis] = 7; break;
                    case 'w': case 'W': *_rel_axis[axis] = 8; break;
                }
            }
        }
    }

    hal_ready(comp_id);
    return 0;
}




void rtapi_app_exit(void)
{
#if FOR_MACHINEKIT
    hal_remove_vtable(vtable_id);
#endif
    hal_exit(comp_id);
}
