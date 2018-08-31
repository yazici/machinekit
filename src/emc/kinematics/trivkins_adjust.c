/********************************************************************
* Description: trivkins_adjust.c
*   Trivial kinematics for 3 axis Cartesian machine
*   with position adjustment
********************************************************************/

#include "kinematics.h"     /* these decls */

#include "rtapi.h"          /* RTAPI realtime OS API */
#include "rtapi_app.h"      /* RTAPI realtime module decls */
#include "hal.h"



#define VTVERSION VTKINEMATICS_VERSION1
#define JOINT_CNT_MAX 9
#define JOINT_STEPS_MAX 255

MODULE_LICENSE("GPL");

static int comp_id, vtable_id;
static const char *name = "trivkins_adjust";

hal_float_t **_adjust_data[JOINT_CNT_MAX];
hal_float_t **_step_size;
hal_u32_t **_steps;
hal_u32_t **_base_joint;

static char *base_joint;
RTAPI_MP_STRING(base_joint, "base joint id, comma separated");
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
    hal_u32_t joint, step;
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
    _steps      = hal_malloc(JOINT_CNT_MAX * sizeof(hal_u32_t*));
    _step_size  = hal_malloc(JOINT_CNT_MAX * sizeof(hal_float_t*));
    _base_joint = hal_malloc(JOINT_CNT_MAX * sizeof(hal_u32_t*));
    if ( !_steps || !_step_size || !_base_joint )
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: hal_malloc() failed \n", name);
    }

    // export info pins
    for ( joint = 0, retval = 0; joint < JOINT_CNT_MAX; joint++ )
    {
        retval += hal_pin_u32_newf  (HAL_OUT, &_steps[joint],      comp_id, "%s.%d.steps",      name, joint);
        retval += hal_pin_float_newf(HAL_IN,  &_step_size[joint],  comp_id, "%s.%d.step_size",  name, joint);
        retval += hal_pin_u32_newf  (HAL_IN,  &_base_joint[joint], comp_id, "%s.%d.base_joint", name, joint);
    }
    if ( retval )
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: HAL pins export failed \n", name);
        return -1;
    }

    // parse parameter string
    if ( steps != NULL )
    {
        for ( joint = 0, data = steps; (token = strtok(data, ",")) != NULL; joint++ )
        {
            if ( data != NULL ) data = NULL;

            // get steps count
            *_steps[joint] = (hal_u32_t) strtoul(token, NULL, 10);
            if ( *_steps[joint] <= 0 ) continue;
            if ( *_steps[joint] > JOINT_STEPS_MAX ) *_steps[joint] = JOINT_STEPS_MAX;

            // shared memory allocation
            _adjust_data[joint] = hal_malloc(*_steps[joint] * sizeof(hal_float_t*));
            if ( ! *_adjust_data[joint] )
            {
                rtapi_print_msg(RTAPI_MSG_ERR, "%s: hal_malloc() failed \n", name);
                return -1;
            }

            // export step pins
            for ( step = 0, retval = 0; step < *_steps[joint]; step++ )
            {
                retval += hal_pin_float_newf(HAL_IN, &_adjust_data[joint][step],
                              comp_id, "%s.%d.s%d", name, joint, step);
            }
            if ( retval )
            {
                rtapi_print_msg(RTAPI_MSG_ERR, "%s: HAL pins export failed \n", name);
                return -1;
            }
        }
    }

    // parse parameter string
    if ( step_size != NULL )
    {
        for ( joint = 0, data = step_size; (token = strtok(data, ",")) != NULL; joint++ )
        {
            if ( data != NULL ) data = NULL;
            *_step_size[joint] = (hal_float_t) strtod(token, NULL);
        }
    }

    // parse parameter string
    if ( base_joint != NULL )
    {
        for ( joint = 0, data = base_joint; (token = strtok(data, ",")) != NULL; joint++ )
        {
            if ( data != NULL ) data = NULL;

            // parse joint id as number
            if ( token[0] >= '0' && token[0] <= '9' )
            {
                *_base_joint[joint] = (hal_u32_t) strtol(token, NULL, 10);
                if ( *_base_joint[joint] >= JOINT_CNT_MAX ) *_base_joint[joint] = JOINT_CNT_MAX - 1;
            }
            // parse joint id as char
            else
            {
                switch ( token[0] )
                {
                    case 'x': case 'X': *_base_joint[joint] = 0; break;
                    case 'y': case 'Y': *_base_joint[joint] = 1; break;
                    case 'z': case 'Z': *_base_joint[joint] = 2; break;
                    case 'a': case 'A': *_base_joint[joint] = 3; break;
                    case 'b': case 'B': *_base_joint[joint] = 4; break;
                    case 'c': case 'C': *_base_joint[joint] = 5; break;
                    case 'u': case 'U': *_base_joint[joint] = 6; break;
                    case 'v': case 'V': *_base_joint[joint] = 7; break;
                    case 'w': case 'W': *_base_joint[joint] = 8; break;
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
