/********************************************************************
* Description: trivkins_adjust.c
*   Trivial kinematics for 3 axis Cartesian machine
*   with position adjustment
********************************************************************/

#include "kinematics.h"     /* these decls */

#include "rtapi.h"          /* RTAPI realtime OS API */
#include "rtapi_app.h"      /* RTAPI realtime module decls */
#include "hal.h"

#include <string.h>
#include <stdint.h>

#include <linux/fs.h>
#include <asm/segment.h>
#include <asm/uaccess.h>
#include <linux/buffer_head.h>



#define VTVERSION VTKINEMATICS_VERSION1

MODULE_LICENSE("GPL");

static int comp_id, vtable_id;
static const char *name = "trivkins_adjust";

static char *x_adj_file;
RTAPI_MP_STRING(x_adj_file, "X axis adjustment file path");
static char *y_adj_file;
RTAPI_MP_STRING(y_adj_file, "Y axis adjustment file path");
static char *z_adj_file;
RTAPI_MP_STRING(z_adj_file, "Z axis adjustment file path");




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




struct file *file_open(const char *path, int flags, int rights)
{
    struct file *filp = NULL;
    mm_segment_t oldfs;
    int err = 0;

    oldfs = get_fs();
    set_fs(get_ds());
    filp = filp_open(path, flags, rights);
    set_fs(oldfs);
    if (IS_ERR(filp)) {
        err = PTR_ERR(filp);
        return NULL;
    }
    return filp;
}

void file_close(struct file *file)
{
    filp_close(file, NULL);
}

int file_read(struct file *file, unsigned long long offset, unsigned char *data, unsigned int size)
{
    mm_segment_t oldfs;
    int ret;

    oldfs = get_fs();
    set_fs(get_ds());

    ret = vfs_read(file, data, size, &offset);

    set_fs(oldfs);
    return ret;
}

int file_write(struct file *file, unsigned long long offset, unsigned char *data, unsigned int size)
{
    mm_segment_t oldfs;
    int ret;

    oldfs = get_fs();
    set_fs(get_ds());

    ret = vfs_write(file, data, size, &offset);

    set_fs(oldfs);
    return ret;
}

int file_sync(struct file *file)
{
    vfs_fsync(file, 0);
    return 0;
}




int rtapi_app_main(void)
{
    comp_id = hal_init(name);
    if ( comp_id < 0 ) return -1;

    vtable_id = hal_export_vtable(name, VTVERSION, &vtk, comp_id);
    if ( vtable_id < 0 )
    {
        rtapi_print_msg( RTAPI_MSG_ERR,
            "%s: ERROR: hal_export_vtable(%s,%d,%p) failed: %d\n",
            name, name, VTVERSION, &vtk, vtable_id );
        return -1;
    }

    if (x_adj_file != NULL)
    {

    }

    hal_ready(comp_id);
    return 0;
}

void rtapi_app_exit(void)
{
    hal_remove_vtable(vtable_id);
    hal_exit(comp_id);
}
