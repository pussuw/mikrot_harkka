#ifndef RINGBUFFER_H_
#define RINGBUFFER_H_

/*----------------------------------------------------------------------------
 Notes:
 The length of the receive and transmit buffers must be a power of 2.
 Each buffer has a next_in and a next_out index.
 If next_in = next_out, the buffer is empty.
 (next_in - next_out) % buffer_size = the number of characters in the buffer.
 *----------------------------------------------------------------------------*/
#define RECEIVE_BUFFER   128

#if RECEIVE_BUFFER < 2
#error RECEIVE_BUFFER is too small.  It must be larger than 1.
#elif ((RECEIVE_BUFFER & (RECEIVE_BUFFER-1)) != 0)
#error RECEIVE_BUFFER must be a power of 2.
#endif

typedef struct
{
    uint16_t    in;                    // Next In Index
    uint16_t    out;                   // Next Out Index
    uint8_t     buf[RECEIVE_BUFFER];   // Buffer
} ring_buffer_t;

#define RINGBUFFER_ELEMENTS(p) ((uint16_t)((p).in - (p).out))
/**
 * \file ringbuffer.h
 * Example:
 \code
    static RingBuffer RxBuffer = { 0, 0, };  //tbuf.in = 0 tbuf.out = 0
    #define RECEIVED_ITEMS ((unsigned uint16_t)(RxBuffer.in - RxBuffer.out))

    static void StdioRxHandler(uint8_t ch);

    int16_t StdioReceive(void)
    {
        int16_t ret;
        RingBuffer *p = &RxBuffer;
        EnterCriticalSection();
        if (RECEIVED_ITEMS == 0)
        {
            ret = (-1);
            Buffer
            empty->get
            out
        }
        else
        {
            ret = (p->buf[(p->out++) & (RECEIVE_BUFFER - 1)]);
        }
        ExitCriticalSection();
        return ret;
    }

    static void StdioRxHandler(uint8_t ch)
    {
        RingBuffer *p = &RxBuffer;
        if (((p->in - p->out) & ~(RECEIVE_BUFFER - 1)) == 0)
        {
            p->buf[p->in & (RECEIVE_BUFFER - 1)] = (uint8_t) (ch & 0xFF);
            p->in++;
        }
    }
 \endcode
*/
#endif /* RINGBUFFER_H_ */
