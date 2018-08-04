/**
 * @file    api.h
 * @brief   Allwinner H3 SoC ARISC CNC firmware API
 *
 * @note    Firmware:   https://github.com/orangecnc/h3_arisc_firmware
 * @note    API:        https://github.com/orangecnc/h3_arisc_api
 *
 * @author  Mikhail Vydrenko (mikhail@vydrenko.ru)
 */

#ifndef _API_H
#define _API_H

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




#define GPIO_PINS_CNT 32 ///< number of GPIO port pins

/// the GPIO port names
enum { PA, PB, PC, PD, PE, PF, PG, PL };

/// the GPIO pin states
enum { LOW, HIGH };

/// the message types
enum
{
    GPIO_MSG_SETUP_FOR_OUTPUT = 0x10,
    GPIO_MSG_SETUP_FOR_INPUT,

    GPIO_MSG_PIN_GET,
    GPIO_MSG_PIN_SET,
    GPIO_MSG_PIN_CLEAR,

    GPIO_MSG_PORT_GET,
    GPIO_MSG_PORT_SET,
    GPIO_MSG_PORT_CLEAR
};

/// the message data access
struct gpio_msg_port_t      { uint32_t port; };
struct gpio_msg_port_pin_t  { uint32_t port; uint32_t pin;  };
struct gpio_msg_port_mask_t { uint32_t port; uint32_t mask; };
struct gpio_msg_state_t     { uint32_t state; };




#define PULSGEN_CH_CNT 32  ///< maximum number of pulse generator channels

/// messages types
enum
{
    PULSGEN_MSG_PIN_SETUP = 0x20,
    PULSGEN_MSG_TASK_SETUP,
    PULSGEN_MSG_TASK_ABORT,
    PULSGEN_MSG_TASK_STATE,
    PULSGEN_MSG_TASK_TOGGLES
};

/// the message data access
struct pulsgen_msg_pin_setup_t { uint32_t ch; uint32_t port; uint32_t pin; uint32_t inverted; };
struct pulsgen_msg_task_setup_t { uint32_t ch; uint32_t toggles;
    uint32_t pin_setup_time; uint32_t pin_hold_time; uint32_t start_delay; };
struct pulsgen_msg_ch_t { uint32_t ch; };
struct pulsgen_msg_state_t { uint32_t state; };
struct pulsgen_msg_toggles_t { uint32_t toggles; };




#define ENCODER_CH_CNT 8 ///< maximum number of encoder counter channels

enum { PHASE_A, PHASE_B, PHASE_Z };
enum { PH_A, PH_B, PH_Z };

/// messages types
enum
{
    ENCODER_MSG_PIN_SETUP = 0x30,
    ENCODER_MSG_SETUP,
    ENCODER_MSG_STATE_SET,
    ENCODER_MSG_STATE_GET,
    ENCODER_MSG_COUNTS_SET,
    ENCODER_MSG_COUNTS_GET
};

/// the message data access
struct encoder_msg_ch_t { uint32_t ch; };
struct encoder_msg_pin_setup_t { uint32_t ch; uint32_t phase; uint32_t port; uint32_t pin; };
struct encoder_msg_setup_t { uint32_t ch; uint32_t using_B; uint32_t using_Z; };
struct encoder_msg_state_set_t { uint32_t ch; uint32_t state; };
struct encoder_msg_counts_set_t { uint32_t ch; int32_t counts; };
struct encoder_msg_state_get_t { uint32_t state; };
struct encoder_msg_counts_get_t { int32_t counts; };




// private vars

static struct msg_t * msg_arisc[MSG_MAX_CNT] = {0};
static struct msg_t * msg_arm[MSG_MAX_CNT] = {0};
static uint8_t msg_buf[MSG_LEN] = {0};

static uint32_t *vrt_block_addr = 0;










// public methods

int32_t mem_init(uint8_t cpu_id)
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
    if ( (mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0 ) return -1;

    // calculate phy memory block start
    vrt_offset = msg_block_addr % PHY_MEM_BLOCK_SIZE;
    phy_block_addr = msg_block_addr - vrt_offset;

    // make a block of phy memory visible in our user space
    vrt_block_addr = mmap(NULL, 2*MSG_BLOCK_SIZE, PROT_READ | PROT_WRITE,
        MAP_SHARED, mem_fd, phy_block_addr);

    // exit program if mmap is failed
    if (vrt_block_addr == MAP_FAILED) return -2;

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

void mem_deinit(void)
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




/**
 * @brief   setup encoder pin for the selected channel and phase
 *
 * @param   c           channel id
 * @param   phase       PHASE_A..PHASE_Z
 * @param   port        GPIO port number
 * @param   pin         GPIO pin number
 *
 * @retval  none
 */
void encoder_pin_setup(uint8_t c, uint8_t phase, uint8_t port, uint8_t pin)
{
    struct encoder_msg_pin_setup_t tx = *((struct encoder_msg_pin_setup_t *) &msg_buf);

    tx.ch = c;
    tx.phase = phase;
    tx.port = port;
    tx.pin = pin;

    msg_send(ENCODER_MSG_PIN_SETUP, (uint8_t*)&tx, 4*4, 0);
}

/**
 * @brief   setup selected channel of encoder counter
 *
 * @param   c           channel id
 * @param   using_B     use phase B input?
 * @param   using_Z     use phase Z index input?
 *
 * @retval  none
 */
void encoder_setup(uint8_t c, uint8_t using_B, uint8_t using_Z)
{
    struct encoder_msg_setup_t tx = *((struct encoder_msg_setup_t *) &msg_buf);

    tx.ch = c;
    tx.using_B = using_B;
    tx.using_Z = using_Z;

    msg_send(ENCODER_MSG_SETUP, (uint8_t*)&tx, 3*4, 0);
}

/**
 * @brief   enable/disable selected channel of encoder counter
 * @param   c       channel id
 * @retval  none
 */
void encoder_state_set(uint8_t c, uint8_t state)
{
    struct encoder_msg_state_set_t tx = *((struct encoder_msg_state_set_t *) &msg_buf);

    tx.ch = c;
    tx.state = state;

    msg_send(ENCODER_MSG_STATE_SET, (uint8_t*)&tx, 2*4, 0);
}

/**
 * @brief   change number of counts for the selected channel
 * @param   c       channel id
 * @param   counts  new value for encoder channel counts
 * @retval  none
 */
void encoder_counts_set(uint8_t c, int32_t counts)
{
    struct encoder_msg_counts_set_t tx = *((struct encoder_msg_counts_set_t *) &msg_buf);

    tx.ch = c;
    tx.counts = counts;

    msg_send(ENCODER_MSG_COUNTS_SET, (uint8_t*)&tx, 2*4, 0);
}

/**
 * @brief   get state for the selected channel
 *
 * @param   c   channel id
 *
 * @retval  0   (channel is disabled)
 * @retval  1   (channel is enabled)
 */
uint8_t encoder_state_get(uint8_t c)
{
    struct encoder_msg_ch_t tx = *((struct encoder_msg_ch_t *) &msg_buf);
    struct encoder_msg_state_get_t rx = *((struct encoder_msg_state_get_t *) &msg_buf);

    tx.ch = c;

    msg_send(ENCODER_MSG_STATE_GET, (uint8_t*)&tx, 1*4, 0);

    // finite loop, only 999999 tries to read an answer
    uint32_t n = 0;
    for ( n = 999999; n--; )
    {
        if ( msg_read(ENCODER_MSG_STATE_GET, (uint8_t*)&rx, 0) < 0 ) continue;
        else return rx.state;
    }

    return 0;
}

/**
 * @brief   get current counts for the selected channel
 * @param   c   channel id
 * @retval  signed 32-bit number
 */
int32_t encoder_counts_get(uint8_t c)
{
    struct encoder_msg_ch_t tx = *((struct encoder_msg_ch_t *) &msg_buf);
    struct encoder_msg_counts_get_t rx = *((struct encoder_msg_counts_get_t *) &msg_buf);

    tx.ch = c;

    msg_send(ENCODER_MSG_COUNTS_GET, (uint8_t*)&tx, 1*4, 0);

    // finite loop, only 999999 tries to read an answer
    uint32_t n = 0;
    for ( n = 999999; n--; )
    {
        if ( msg_read(ENCODER_MSG_COUNTS_GET, (uint8_t*)&rx, 0) < 0 ) continue;
        else return rx.counts;
    }

    return 0;
}




/**
 * @brief   setup GPIO pin for the selected channel
 *
 * @param   c           channel id
 * @param   port        GPIO port number
 * @param   pin         GPIO pin number
 * @param   inverted    invert pin state?
 *
 * @retval  none
 */
void pulsgen_pin_setup(uint8_t c, uint8_t port, uint8_t pin, uint8_t inverted)
{
    struct pulsgen_msg_pin_setup_t tx = *((struct pulsgen_msg_pin_setup_t *) &msg_buf);

    tx.ch = c;
    tx.port = port;
    tx.pin = pin;
    tx.inverted = inverted;

    msg_send(PULSGEN_MSG_PIN_SETUP, (uint8_t*)&tx, 4*4, 0);
}

/**
 * @brief   setup a new task for the selected channel
 *
 * @param   c               channel id
 * @param   toggles         number of pin state changes
 * @param   pin_setup_time  pin state setup_time (in nanoseconds)
 * @param   pin_hold_time   pin state hold_time (in nanoseconds)
 * @param   start_delay     task start delay (in nanoseconds)
 *
 * @retval  none
 */
void pulsgen_task_setup
(
    uint32_t c,
    uint32_t toggles,
    uint32_t pin_setup_time,
    uint32_t pin_hold_time,
    uint32_t start_delay
)
{
    struct pulsgen_msg_task_setup_t tx = *((struct pulsgen_msg_task_setup_t *) &msg_buf);

    tx.ch = c;
    tx.toggles = toggles;
    tx.pin_setup_time = pin_setup_time;
    tx.pin_hold_time = pin_hold_time;
    tx.start_delay = start_delay;

    msg_send(PULSGEN_MSG_TASK_SETUP, (uint8_t*)&tx, 5*4, 0);
}

/**
 * @brief   abort current task for the selected channel
 * @param   c       channel id
 * @retval  none
 */
void pulsgen_task_abort(uint8_t c)
{
    struct pulsgen_msg_ch_t tx = *((struct pulsgen_msg_ch_t *) &msg_buf);

    tx.ch = c;

    msg_send(PULSGEN_MSG_TASK_ABORT, (uint8_t*)&tx, 1*4, 0);
}

/**
 * @brief   get current task state for the selected channel
 *
 * @param   c   channel id
 *
 * @retval  0   (channel have no task)
 * @retval  1   (channel have a task)
 */
uint8_t pulsgen_task_state(uint8_t c)
{
    struct pulsgen_msg_ch_t tx = *((struct pulsgen_msg_ch_t *) &msg_buf);
    struct pulsgen_msg_state_t rx = *((struct pulsgen_msg_state_t *) &msg_buf);

    tx.ch = c;

    msg_send(PULSGEN_MSG_TASK_STATE, (uint8_t*)&tx, 1*4, 0);

    // finite loop, only 999999 tries to read an answer
    uint32_t n = 0;
    for ( n = 999999; n--; )
    {
        if ( msg_read(PULSGEN_MSG_TASK_STATE, (uint8_t*)&rx, 0) < 0 ) continue;
        else return rx.state;
    }

    return 0;
}

/**
 * @brief   get current pin state changes since task start
 * @param   c   channel id
 * @retval  0..0xFFFFFFFF
 */
uint32_t pulsgen_task_toggles(uint8_t c)
{
    struct pulsgen_msg_ch_t tx = *((struct pulsgen_msg_ch_t *) &msg_buf);
    struct pulsgen_msg_toggles_t rx = *((struct pulsgen_msg_toggles_t *) &msg_buf);

    tx.ch = c;

    msg_send(PULSGEN_MSG_TASK_TOGGLES, (uint8_t*)&tx, 1*4, 0);

    // finite loop, only 999999 tries to read an answer
    uint32_t n = 0;
    for ( n = 999999; n--; )
    {
        if ( msg_read(PULSGEN_MSG_TASK_TOGGLES, (uint8_t*)&rx, 0) < 0 ) continue;
        else return rx.toggles;
    }

    return 0;
}




/**
 * @brief   set pin mode to OUTPUT
 * @param   port    GPIO port number    (0 .. GPIO_PORTS_CNT)
 * @param   pin     GPIO pin number     (0 .. GPIO_PINS_CNT)
 * @retval  none
 */
void gpio_pin_setup_for_output(uint32_t port, uint32_t pin)
{
    struct gpio_msg_port_pin_t tx = *((struct gpio_msg_port_pin_t *) &msg_buf);

    tx.port = port;
    tx.pin  = pin;

    msg_send(GPIO_MSG_SETUP_FOR_OUTPUT, (uint8_t*)&tx, 2*4, 0);
}

/**
 * @brief   set pin mode to INPUT
 * @param   port    GPIO port number    (0 .. GPIO_PORTS_CNT)
 * @param   pin     GPIO pin number     (0 .. GPIO_PINS_CNT)
 * @retval  none
 */
void gpio_pin_setup_for_input(uint32_t port, uint32_t pin)
{
    struct gpio_msg_port_pin_t tx = *((struct gpio_msg_port_pin_t *) &msg_buf);

    tx.port = port;
    tx.pin  = pin;

    msg_send(GPIO_MSG_SETUP_FOR_INPUT, (uint8_t*)&tx, 2*4, 0);
}

/**
 * @brief   get pin state
 * @param   port    GPIO port number    (0 .. GPIO_PORTS_CNT)
 * @param   pin     GPIO pin number     (0 .. GPIO_PINS_CNT)
 * @retval  1 (HIGH)
 * @retval  0 (LOW)
 */
uint32_t gpio_pin_get(uint32_t port, uint32_t pin)
{
    struct gpio_msg_port_pin_t tx = *((struct gpio_msg_port_pin_t *) &msg_buf);
    struct gpio_msg_state_t rx = *((struct gpio_msg_state_t *) &msg_buf);

    tx.port = port;
    tx.pin = pin;

    msg_send(GPIO_MSG_PIN_GET, (uint8_t*)&tx, 2*4, 0);

    // finite loop, only 999999 tries to read an answer
    uint32_t n = 0;
    for ( n = 999999; n--; )
    {
        if ( msg_read(GPIO_MSG_PIN_GET, (uint8_t*)&rx, 0) < 0 ) continue;
        else return rx.state;
    }

    return 0;
}

/**
 * @brief   set pin state to HIGH (1)
 * @param   port    GPIO port number    (0 .. GPIO_PORTS_CNT)
 * @param   pin     GPIO pin number     (0 .. GPIO_PINS_CNT)
 * @retval  none
 */
void gpio_pin_set(uint32_t port, uint32_t pin)
{
    struct gpio_msg_port_pin_t tx = *((struct gpio_msg_port_pin_t *) &msg_buf);

    tx.port = port;
    tx.pin  = pin;

    msg_send(GPIO_MSG_PIN_SET, (uint8_t*)&tx, 2*4, 0);
}

/**
 * @brief   set pin state to LOW (0)
 * @param   port    GPIO port number    (0 .. GPIO_PORTS_CNT)
 * @param   pin     GPIO pin number     (0 .. GPIO_PINS_CNT)
 * @retval  none
 */
void gpio_pin_clear(uint32_t port, uint32_t pin)
{
    struct gpio_msg_port_pin_t tx = *((struct gpio_msg_port_pin_t *) &msg_buf);

    tx.port = port;
    tx.pin  = pin;

    msg_send(GPIO_MSG_PIN_CLEAR, (uint8_t*)&tx, 2*4, 0);
}

/**
 * @brief   get port state
 * @param   port    GPIO port number (0 .. GPIO_PORTS_CNT)
 * @note    each bit value of returned value represents port pin state
 * @retval  0 .. 0xFFFFFFFF
 */
uint32_t gpio_port_get(uint32_t port)
{
    struct gpio_msg_port_t tx = *((struct gpio_msg_port_t *) &msg_buf);
    struct gpio_msg_state_t rx = *((struct gpio_msg_state_t *) &msg_buf);

    tx.port = port;

    msg_send(GPIO_MSG_PORT_GET, (uint8_t*)&tx, 1*4, 0);

    // finite loop, only 999999 tries to read an answer
    uint32_t n = 0;
    for ( n = 999999; n--; )
    {
        if ( msg_read(GPIO_MSG_PORT_GET, (uint8_t*)&rx, 0) < 0 ) continue;
        else return rx.state;
    }

    return 0;
}

/**
 * @brief   set port pins state by mask
 *
 * @param   port    GPIO port number        (0 .. GPIO_PORTS_CNT)
 * @param   mask    GPIO pins mask to set   (0 .. 0xFFFFFFFF) \n\n
 *                  mask examples: \n\n
 *                      mask = 0xFFFFFFFF (0b11111111111111111111111111111111) means <b>set all pins state to 1 (HIGH)</b> \n
 *                      mask = 0x00000001 (0b1) means <b>set pin 0 state to 1 (HIGH)</b> \n
 *                      mask = 0x0000000F (0b1111) means <b>set pins 0,1,2,3 states to 1 (HIGH)</b>
 *
 * @retval  none
 */
void gpio_port_set(uint32_t port, uint32_t mask)
{
    struct gpio_msg_port_mask_t tx = *((struct gpio_msg_port_mask_t *) &msg_buf);

    tx.port = port;
    tx.mask = mask;

    msg_send(GPIO_MSG_PORT_SET, (uint8_t*)&tx, 2*4, 0);
}

/**
 * @brief   clear port pins state by mask
 *
 * @param   port    GPIO port number        (0 .. GPIO_PORTS_CNT)
 * @param   mask    GPIO pins mask to clear (0 .. 0xFFFFFFFF) \n\n
 *                  mask examples: \n\n
 *                  mask = 0xFFFFFFFF (0b11111111111111111111111111111111) means <b>set all pins state to 0 (LOW)</b> \n
 *                  mask = 0x00000003 (0b11) means <b>set pins 0,1 states to 0 (LOW)</b> \n
 *                  mask = 0x00000008 (0b1000) means <b>set pin 3 state to 0 (LOW)</b>
 *
 * @retval  none
 */
void gpio_port_clear(uint32_t port, uint32_t mask)
{
    struct gpio_msg_port_mask_t tx = *((struct gpio_msg_port_mask_t *) &msg_buf);

    tx.port = port;
    tx.mask = mask;

    msg_send(GPIO_MSG_PORT_CLEAR, (uint8_t*)&tx, 2*4, 0);
}




#endif
