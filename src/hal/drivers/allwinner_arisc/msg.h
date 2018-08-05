/**
 * @file    msg.h
 * @brief   MGS related data
 * @author  Mikhail Vydrenko (mikhail@vydrenko.ru)
 */

#ifndef _MSG_H
#define _MSG_H

#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <cpu.h>




// public data

#define PHY_MEM_BLOCK_SIZE      4096
#define ARISC_CONF_SIZE         2048

#define MSG_BLOCK_SIZE          4096
#define MSG_CPU_BLOCK_SIZE      2048
#define MSG_MAX_CNT             32
#define MSG_MAX_LEN             (MSG_CPU_BLOCK_SIZE / MSG_MAX_CNT)
#define MSG_LEN                 (MSG_MAX_LEN - 4)

#pragma pack(push, 1)
struct msg_t
{
    uint8_t length;
    uint8_t type;
    uint8_t locked; // actually not used at this moment
    uint8_t unread;
    uint8_t msg[MSG_LEN];
};
#pragma pack(pop)








// private vars

static struct msg_t * msg_arisc[MSG_MAX_CNT] = {0};
static struct msg_t * msg_arm[MSG_MAX_CNT] = {0};
static uint8_t msg_buf[MSG_LEN] = {0};

static uint32_t *vrt_block_addr = 0;









// public methods

int32_t msg_mem_init
(
    uint8_t cpu_id,
    const char *comp_name
)
{
    int32_t     mem_fd;
    uint32_t    vrt_offset = 0;
    off_t       phy_block_addr = 0;
    int32_t     m = 0;
    uint32_t    msg_block_addr = cpu_data[cpu_id].ARISC_addr +
                                 cpu_data[cpu_id].ARISC_size -
                                 ARISC_CONF_SIZE -
                                 MSG_BLOCK_SIZE;

    // open physical memory file
    if ( (mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0 )
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: [MSG] can't open /dev/mem file\n", comp_name);
        return -1;
    }

    // calculate phy memory block start
    vrt_offset = msg_block_addr % PHY_MEM_BLOCK_SIZE;
    phy_block_addr = msg_block_addr - vrt_offset;

    // make a block of phy memory visible in our user space
    vrt_block_addr = mmap(NULL, 2*MSG_BLOCK_SIZE, PROT_READ | PROT_WRITE,
        MAP_SHARED, mem_fd, phy_block_addr);

    // exit program if mmap is failed
    if (vrt_block_addr == MAP_FAILED)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: [MSG] mmap() failed\n", comp_name);
        return -2;
    }

    // no need to keep phy memory file open after mmap
    close(mem_fd);

    // adjust offset to correct value
    vrt_block_addr += (vrt_offset/4);

    // assign messages pointers
    for ( m = 0; m < MSG_MAX_CNT; ++m )
    {
        msg_arisc[m] = (struct msg_t *) (vrt_block_addr + (m * MSG_MAX_LEN)/4);
        msg_arm[m]   = (struct msg_t *) (vrt_block_addr + (m * MSG_MAX_LEN + MSG_CPU_BLOCK_SIZE)/4);
    }

    return 0;
}

void msg_mem_deinit(void)
{
    munmap(vrt_block_addr, 2*MSG_BLOCK_SIZE);
}




/**
 * @brief   read a message from the ARISC cpu
 *
 * @param   type    user defined message type (0..0xFF)
 * @param   msg     pointer to the message buffer
 * @param   bswap   0 - if you sending an array of 32bit numbers, 1 - for the text
 *
 * @retval   0 (message read)
 * @retval  -1 (message not read)
 */
int8_t msg_read(uint8_t type, uint8_t * msg, uint8_t bswap)
{
    static uint8_t last = 0;
    static uint8_t m = 0;
    static uint8_t i = 0;
    static uint32_t * link;

    // find next unread message
    for ( i = MSG_MAX_CNT, m = last; i--; )
    {
        // process message only of current type
        if ( msg_arisc[m]->unread && msg_arisc[m]->type == type )
        {
            if ( bswap )
            {
                // swap message data bytes for correct reading by ARM
                link = ((uint32_t*) &msg_arisc[m]->msg);
                for ( i = msg_arisc[m]->length / 4 + 1; i--; link++ )
                {
                    *link = __bswap_32(*link);
                }
            }

            // copy message to the buffer
            memcpy(msg, &msg_arisc[m]->msg, msg_arisc[m]->length);

            // message read
            msg_arisc[m]->unread = 0;
            last = m;
            return msg_arisc[m]->length;
        }

        ++m;
        if ( m >= MSG_MAX_CNT ) m = 0;
    }

    return -1;
}

/**
 * @brief   send a message to the ARISC cpu
 *
 * @param   type    user defined message type (0..0xFF)
 * @param   msg     pointer to the message buffer
 * @param   length  the length of a message (0..MSG_LEN) )
 * @param   bswap   0 - if you sending an array of 32bit numbers, 1 - for the text
 *
 * @retval   0 (message sent)
 * @retval  -1 (message not sent)
 */
int8_t msg_send(uint8_t type, uint8_t * msg, uint8_t length, uint8_t bswap)
{
    static uint8_t last = 0;
    static uint8_t m = 0;
    static uint8_t i = 0;
    static uint32_t * link;

    // find next free message slot
    for ( i = MSG_MAX_CNT, m = last; i--; )
    {
        // sending message
        if ( !msg_arm[m]->unread )
        {
            // copy message to the buffer
            memset( (uint8_t*)((uint8_t*)&msg_arm[m]->msg + length/4*4), 0, 4);
            memcpy(&msg_arm[m]->msg, msg, length);

            if ( bswap )
            {
                // swap message data bytes for correct reading by ARISC
                link = ((uint32_t*) &msg_arm[m]->msg);
                for ( i = length / 4 + 1; i--; link++ )
                {
                    *link = __bswap_32(*link);
                }
            }

            // set message data
            msg_arm[m]->type   = type;
            msg_arm[m]->length = length;
            msg_arm[m]->unread = 1;

            // message sent
            last = m;
            return 0;
        }

        ++m;
        if ( m >= MSG_MAX_CNT ) m = 0;
    }

    // message not sent
    return -1;
}




#endif
