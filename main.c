/*
 * main.c
 *
 *  Created on: 26.4.2014
 *      Author: Ville
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>

/* Global defines */
#ifndef __cplusplus
#undef false
#undef true
#undef bool
typedef enum {false, true} bool;
#endif

#define __enable_irq()          sei()
#define __disable_irq()         cli()

/* Serial port */
#define MCU_FREQUENCY           16000000ul
#define BAUDRATE                9600ul
#define PRESCALER(BAUD, FREQ)   ((FREQ / (BAUD * 16u)) - 1)
#define RECEIVE_BUFFER          128

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

/* Serial RX ringbuffer */
static ring_buffer_t        m_uart_rx_buf = {0, 0, };

/* Pause feature */
static volatile bool        m_paused = false;
#define pause_led_on()      do{PORTB |= (1 << 5);}while(0)
#define pause_led_off()     do{PORTB &= ~(1 << 5);}while(0)

/* Clear pin on compare match */
#define output_disable()    do{TCCR2A &= ~(1 << COM2A1);}while(0)
#define output_enable()     do{TCCR2A |= (1 << COM2A1);}while(0)

/* PWM/PID values */
#define TO_DECIMAL_COEFF(d) ((d) * 10)
#define TO_RAW_COEFF(f)     ((f) / 10)
#define P_COEFF             (1.0)
#define I_COEFF             (0.4)
#define D_COEFF             (0.4)

typedef struct
{
    struct
    {
        /* Coefficients */
        uint8_t p;
        uint8_t i;
        uint8_t d;
    }coeffs;
    int16_t d_part;
    int16_t i_part;
}pid_controller_t;

/* Initialize PID Controller */
#define reset_pid(p)        ((p).i_part = 0)
static pid_controller_t     m_pid_controller = {{TO_DECIMAL_COEFF(P_COEFF),
                                                 TO_DECIMAL_COEFF(I_COEFF),
                                                 TO_DECIMAL_COEFF(D_COEFF)},
                                                0, 0};

/* Define ADC / PWM characteristics */
#define max(a, b)           ((a) > (b) ? (a) : (b))
#define min(a, b)           ((a) < (b) ? (a) : (b))
#define adc_ref_mv          5000u
#define adc_max             1024u
#define adc_to_mv(adc)      adc == 0 ? 0 : \
                            ((uint32_t)((uint32_t)(adc) * adc_ref_mv) / adc_max)
#define pwm_max_volt        5000u
#define pwm_max_value       255u
#define mv_to_pwm(mv)       mv == 0 ? 0 : \
                            ((uint32_t)((uint32_t)(mv) * pwm_max_value) / pwm_max_volt)
#define pwm_fb_enable()     do{ADCSRA |= (1 << ADIE); ADCSRA |= (1 << ADSC);}while(0)
#define pwm_fb_disable()    do{ADCSRA &= ~(1 << ADIE);}while(0)

/* Target voltage in millivolts */
static volatile int16_t     m_pwm_target_value = 0;
static volatile uint8_t     m_pwm_set_value = 0;
static volatile uint8_t     m_print_buf[256];

int16_t pid_run(int16_t target,
                int16_t meas,
                pid_controller_t * pid)
{
    /* Error : -65535...65535 */
    int32_t error;
    int32_t p, d, i, sum;
    error = target - meas;
    /* D-part: Cannot over/underflow (0xFF * 0xFFFF) = 0xFFFFFF */
    p = pid->coeffs.p * error;
    /* I-part: Error can over/underflow... */
    i = pid->i_part + error;
    /* But cap values at -32767...32767 */
    i = max(i, -INT16_MAX);
    i = min(i, INT16_MAX);
    /* Store error and calculate I-part */
    pid->i_part = i;
    i *= pid->coeffs.i;
    /* D-part */
    d = pid->d_part - meas;
    d *= pid->coeffs.d;
    pid->d_part = meas;
    sum = p + i + d;
    sum = TO_RAW_COEFF(sum);
    sum = max(sum, -INT16_MAX);
    sum = min(sum, INT16_MAX);
    return (int16_t)(sum);
}

int16_t serial_read(void)
{
    int16_t ret;
    ring_buffer_t *p = &m_uart_rx_buf;
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

void serve_serial(void)
{
    uint8_t pwm_set_latch = 0, len = 0;
    int16_t ch = serial_read();
    if(ch >= '0' && ch <= '5')
    {
        int16_t new_target = ch - '0';
        if(new_target != m_pwm_target_value)
        {
            __disable_irq();
            reset_pid(m_pid_controller);
            m_pwm_target_value = new_target * 1000u;
            __enable_irq();
        }
    }
    /* For some mysterious reason INT0 starts oscillating on
     * rising edge, when PORTD 2 is driven (serial TX) */
    __disable_irq();
    pwm_set_latch = m_pwm_set_value;
    __enable_irq();
    len = snprintf((char*)m_print_buf,
                   sizeof(m_print_buf),
                   "target:%u\t"
                   "pwm_out:%u\n",
                   m_pwm_target_value,
                   pwm_set_latch);
    serial_write((void*)m_print_buf, len);
}

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

void init(void)
{
    /* GPIO
     * PORTD :
     * 7: OUT (GPI0)
     * 6: OUT (GPIO)
     * 5: OUT (GPIO)
     * 4: OUT (GPIO)
     * 3: OUT (GPIO)
     * 2: IN  (INT0)
     * 1: OUT (UART TXD)
     * 0: IN  (UART RXD) */
    PORTD = 0;
    DDRD = 0b11111010;

    /* PORTB :
     * 7: IN  (XTAL)
     * 6: IN  (XTAL)
     * 5: OUT (LED0)
     * 4: OUT (GPIO)
     * 3: OUT (PWM)
     * 2: OUT (GPIO)
     * 1: OUT (GPIO)
     * 0: OUT (GPIO)
     */
    PORTB = 0;
    DDRB = 0b00111111;

    /* PORTC :
     * 7: NC
     * 6: IN (RST)
     * 5: IN (ADC5)
     * 4: IN (ADC4)
     * 3: IN (ADC3)
     * 2: IN (ADC2)
     * 1: IN (ADC1)
     * 0: IN (ADC0)
     */
    PORTC = 0;
    DDRC = 0b00000000;

    /* INT0 Pin:
     * Falling edge triggers interrupt */
    EIFR |= 0x01;
    EICRA = (1 << ISC01);
    EIMSK = 0x01;
    pause_led_off();

    /* TIMER2 (Fast PWM)
     * Reset timer count
     * Set PWM to 0
     * Enable COMPA interrupt
     * Start timer: No prescaler */
    output_disable();
    TCNT2 = 0;
    OCR2A = 0;
    TIFR2 |= (1 << OCF2A);
    TIMSK2 |= ((1 << OCIE2A) | (1 << TOIE2));
    TCCR2A = ((1 << WGM21) | (1 << WGM20));
    TCCR2B = ((1 << CS20));

    /* ADC
     * Set AD prescaler to 128
     * PWM LPF feedback is connected to ADC channel 0
     * ADC free running mode (triggered by TIM2 ofv)
     * Measures the output voltage of PWM LPF
     * Interrupt for conversion ready
     * ISR runs PID, which sets value for CC2A */
    pwm_fb_disable();
    ADMUX = (1 << REFS0);
    ADCSRA = ((1 << ADEN) |
              (1 << ADPS2) |
              (1 << ADPS1) |
              (1 << ADPS0));
    ADCSRB = 0;
    DIDR0 = (1 << ADC0D);
    ADCSRA |= (1 << ADSC);
    ADCSRA |= (1 << ADSC);
}

int main(void)
{
    /* Initialize everything before enabling output */
    init();
    serial_init();
    /* Finally enable output */
    pwm_fb_enable();
    output_enable();
    __enable_irq();
    while(1)
    {
        serve_serial();
    }
    return 0;
}

ISR(ADC_vect)
{
    uint8_t pwm = 0;
    uint16_t ad_mv;
    int16_t pid_out;
    /* Value is right adjusted */
    ad_mv = ADC;
    ad_mv = adc_to_mv(ad_mv);
    pid_out = pid_run(m_pwm_target_value,
                      ad_mv,
                      &m_pid_controller);
    if(pid_out < 0)
    {
        /* Can't drive negative values */
        pid_out = 0;
    }
    /* No reason to set over maximum voltage */
    pid_out = min(pid_out, pwm_max_volt);
    pwm = mv_to_pwm(pid_out);
    m_pwm_set_value = pwm;
    /* Start new conversion */
    ADCSRA |= (1 << ADSC);
}

ISR(INT0_vect)
{
    PORTB ^= 0x01;
    if(m_paused)
    {
        /* Resume */
        pause_led_off();
        output_enable();
        pwm_fb_enable();
    }
    else
    {
        /* Pause */
        pause_led_on();
        pwm_fb_disable();
        output_disable();
    }
    m_paused = !m_paused;
}

EMPTY_INTERRUPT(TIMER2_COMPA_vect);
ISR(TIMER2_OVF_vect)
{
    /* Set new PWM period when timer count resets */
    OCR2A = m_pwm_set_value;
}


ISR(USART_RX_vect)
{
    ring_buffer_t *p = &m_uart_rx_buf;
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
