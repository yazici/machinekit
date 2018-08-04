/**
 * @file    cpu.h
 * @brief   Allwinner CPUs related data
 * @author  Mikhail Vydrenko (mikhail@vydrenko.ru)
 */

#ifndef _CPU_H
#define _CPU_H




enum
{
    H2_PLUS,
    H3,
    H5,
    A31,
    A64,

    CPU_CNT
};

struct cpu_data_t
{
    char *name;
    uint32_t ARISC_addr;
    uint32_t ARISC_size;
};




static uint8_t cpu_id = H3;
static struct cpu_data_t cpu_data[CPU_CNT] =
{
    {"H2+", 0x00040000, (8+8+32)*1024},
    {"H3",  0x00040000, (8+8+32)*1024},
    {"H5",  0x00040000, (8+8+64)*1024},
    {"A31", 0x00040000, (8+8+64)*1024},
    {"A64", 0x00040000, (8+8+64)*1024},
};




#endif
