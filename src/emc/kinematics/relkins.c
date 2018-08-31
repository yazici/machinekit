/********************************************************************
* Description: trivkins_adjust.c
*   Trivial kinematics for 3 axis Cartesian machine
*   with position adjustment
********************************************************************/

#include "kinematics.h"     /* these decls */

#include "rtapi.h"          /* RTAPI realtime OS API */
#include "rtapi_app.h"      /* RTAPI realtime module decls */
#include "hal.h"
#include <stdlib.h>



#define VTVERSION VTKINEMATICS_VERSION1
#define AXIS_CNT_MAX 9
#define AXIS_STEPS_MAX 255

MODULE_LICENSE("GPL");

static int comp_id, vtable_id;
static const char *name = "relkins";
static const char *axis_name = "XYZABCUVW";

hal_float_t **_adjust_data[AXIS_CNT_MAX];
hal_float_t **_step_size;
hal_u32_t **_steps;
hal_u32_t **_rel;

static char *rel;
RTAPI_MP_STRING(rel, "rel joint id, comma separated");
static char *step_size;
RTAPI_MP_STRING(step_size, "step size, comma separated");
static char *steps;
RTAPI_MP_STRING(steps, "steps count, comma separated");




int
kinematicsForward
(
    const double *                      joints,
    EmcPose *                           pos,
    const KINEMATICS_FORWARD_FLAGS *    fflags,
    KINEMATICS_INVERSE_FLAGS *          iflags
)
{
    pos->tran.x = joints[0];
    pos->tran.y = joints[1];
    pos->tran.z = joints[2];
    pos->a = joints[3];
    pos->b = joints[4];
    pos->c = joints[5];
    pos->u = joints[6];
    pos->v = joints[7];
    pos->w = joints[8];

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
    joints[0] = pos->tran.x;
    joints[1] = pos->tran.y;
    joints[2] = pos->tran.z;
    joints[3] = pos->a;
    joints[4] = pos->b;
    joints[5] = pos->c;
    joints[6] = pos->u;
    joints[7] = pos->v;
    joints[8] = pos->w;

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

static vtkins_t vtk =
{
    .kinematicsForward = kinematicsForward,
    .kinematicsInverse  = kinematicsInverse,
    // .kinematicsHome = kinematicsHome,
    .kinematicsType = kinematicsType
};




int rtapi_app_main(void)
{
    hal_s32_t retval;
    hal_u32_t axis, step, steps_cnt[AXIS_CNT_MAX] = {0};
    hal_s8_t *data, *token;

    // component init
    comp_id = hal_init(name);
    if ( comp_id < 0 ) return -1;

    // export kinematics table
    vtable_id = hal_export_vtable(name, VTVERSION, &vtk, comp_id);
    if ( vtable_id < 0 )
    {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "%s: ERROR: hal_export_vtable(%s,%d,%p) failed: %d\n",
            name, name, VTVERSION, &vtk, vtable_id);
        return -1;
    }

    // shared memory allocation
    _steps      = hal_malloc(AXIS_CNT_MAX * sizeof(hal_u32_t*));
    _step_size  = hal_malloc(AXIS_CNT_MAX * sizeof(hal_float_t*));
    _rel        = hal_malloc(AXIS_CNT_MAX * sizeof(hal_u32_t*));
    if ( !_steps || !_step_size || !_rel )
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: info pins hal_malloc() failed \n", name);
    }

    // parse parameter string
    if ( steps != NULL )
    {
        for ( axis = 0, data = steps; (token = strtok(data, ",")) != NULL; axis++ )
        {
            if ( data != NULL ) data = NULL;

            // get steps count
            steps_cnt[axis] = (hal_u32_t) strtoul(token, NULL, 10);
            if ( steps_cnt[axis] <= 0 ) continue;
            if ( steps_cnt[axis] > AXIS_STEPS_MAX ) steps_cnt[axis] = AXIS_STEPS_MAX;

            // shared memory allocation
            _adjust_data[axis] = hal_malloc(steps_cnt[axis] * sizeof(hal_float_t*));
            if ( !_adjust_data[axis] )
            {
                rtapi_print_msg(RTAPI_MSG_ERR, "%s: step pins hal_malloc() failed \n", name);
                return -1;
            }

            // export step pins
            for ( step = 0, retval = 0; step < steps_cnt[axis]; step++ )
            {
                retval += hal_pin_float_newf(HAL_IN, &_adjust_data[axis][step],
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
        if ( !steps_cnt[axis] ) continue;

        retval += hal_pin_u32_newf  (HAL_OUT, &_steps[axis],     comp_id, "%s.%c.steps",     name, axis_name[axis]);
        retval += hal_pin_float_newf(HAL_IN,  &_step_size[axis], comp_id, "%s.%c.step_size", name, axis_name[axis]);
        retval += hal_pin_u32_newf  (HAL_IN,  &_rel[axis],       comp_id, "%s.%c.rel",      name, axis_name[axis]);

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
        for ( axis = 0, data = step_size; (token = strtok(data, ",")) != NULL; axis++ )
        {
            if ( data != NULL ) data = NULL;
            if ( !steps_cnt[axis] ) continue;

            *_step_size[axis] = (hal_float_t) strtod(token, NULL);
        }
    }

    // parse parameter string
    if ( rel != NULL )
    {
        for ( axis = 0, data = rel; (token = strtok(data, ",")) != NULL; axis++ )
        {
            if ( data != NULL ) data = NULL;
            if ( !steps_cnt[axis] ) continue;

            // parse axis id as number
            if ( token[0] >= '0' && token[0] <= '9' )
            {
                *_rel[axis] = (hal_u32_t) strtol(token, NULL, 10);
                if ( *_rel[axis] >= AXIS_CNT_MAX ) *_rel[axis] = AXIS_CNT_MAX - 1;
            }
            // parse axis id as char
            else
            {
                switch ( token[0] )
                {
                    case 'x': case 'X': *_rel[axis] = 0; break;
                    case 'y': case 'Y': *_rel[axis] = 1; break;
                    case 'z': case 'Z': *_rel[axis] = 2; break;
                    case 'a': case 'A': *_rel[axis] = 3; break;
                    case 'b': case 'B': *_rel[axis] = 4; break;
                    case 'c': case 'C': *_rel[axis] = 5; break;
                    case 'u': case 'U': *_rel[axis] = 6; break;
                    case 'v': case 'V': *_rel[axis] = 7; break;
                    case 'w': case 'W': *_rel[axis] = 8; break;
                }
            }
        }
    }

    hal_ready(comp_id);
    return 0;
}

void rtapi_app_exit(void)
{
    hal_remove_vtable(vtable_id);
    hal_exit(comp_id);
}
