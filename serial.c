/*
 * serial.c
 *
 *  Created on: 27.4.2014
 *      Author: Ville
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include "ringbuffer.h"
#include "globaldef.h"
#include "serial.h"

#define MCU_FREQUENCY           16000000ul
#define BAUDRATE                9600ul
#define PRESCALER(BAUD, FREQ)   ((FREQ / (BAUD * 16u)) - 1)

static ring_buffer_t            uart_rx_buf = {0, 0, };

void serial_init(void)
{
    /* UART:
     * Disable U2X bit (don't double transmission speed)
     * Enable 8bit mode for transmit
     * Enable trasnitter, receiver and character received interrupt */
    UBRR0 = PRESCALER(BAUDRATE, MCU_FREQUENCY);
    UCSR0A = 0xFC;
    UCSR0C = ((1 << UCSZ01) | (1 << UCSZ00));
    UCSR0B = ((1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0));
}

int16_t serial_read(void)
{
    int16_t ret;
    ring_buffer_t *p = &uart_rx_buf;
    __disable_irq();
    if (RINGBUFFER_ELEMENTS(*p) == 0)
    {
        /* Buffer empty */
        ret = (-1);
    }
    else
    {
        /* Return item from tail */
        ret = (p->buf[(p->out++) & (RECEIVE_BUFFER - 1)]);
    }
    __enable_irq();
    return ret;
}

void serial_write(void * buf,
                  uint8_t len)
{
    uint8_t * p = (uint8_t*) buf;
    while(len-- != 0)
    {
        while((UCSR0A & (1 << UDRE0)) == 0)
        {
        }
        UDR0 = *p++;
    }
}

ISR(USART_RX_vect)
{
    ring_buffer_t *p = &uart_rx_buf;
    /* Read byte regardless of buffer state (clears the pending IRQ) */
    uint8_t ch = UDR0;
    /* Is there room in buffer ? */
    if (((p->in - p->out) & ~(RECEIVE_BUFFER - 1)) == 0)
    {
        /* Insert new item to head */
        p->buf[p->in & (RECEIVE_BUFFER - 1)] = (uint8_t) (ch & 0xFF);
        p->in++;
    }
}
