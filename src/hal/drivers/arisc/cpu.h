/**
 * @file    cpu.h
 * @brief   Allwinner CPUs related data
 * @author  Mikhail Vydrenko (mikhail@vydrenko.ru)
 */

#ifndef _CPU_H
#define _CPU_H

#include <stdint.h>
#include <string.h>

#include "rtapi.h"
#include "rtapi_app.h"
#include "hal.h"




enum
{
    H2_PLUS,
    H3,
    H5,

    CPU_CNT
};

struct cpu_data_t
{
    char *name;
    uint32_t ARISC_addr;
    uint32_t ARISC_size;
};








static int8_t *cpu;
RTAPI_MP_STRING(cpu, "CPU name");

static const struct cpu_data_t cpu_data[CPU_CNT] =
{
    {"H2+", 0x00040000, (8+8+32)*1024},
    {"H3",  0x00040000, (8+8+32)*1024},
    {"H5",  0x00040000, (8+8+64)*1024},
};








static uint8_t cpu_id_get()
{
    if ( cpu == NULL ) return H3;

    uint8_t c;
    for ( c = CPU_CNT; c--; )
    {
        if ( !strcmp(cpu, cpu_data[c].name) ) return c;
    }

    return H3;
}




#endif
