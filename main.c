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
#include "globaldef.h"
#include "serial.h"
#include "pid.h"

/* Pause feature */
static volatile bool        m_paused = false;
#define pause_led_on()      do{PORTB |= (1 << 5);}while(0)
#define pause_led_off()     do{PORTB &= ~(1 << 5);}while(0)

/* Toggle led on compare match */
#define output_disable()    do{TCCR2A &= ~(1 << COM2A1);}while(0)
#define output_enable()     do{TCCR2A |= (1 << COM2A1);}while(0)

/* PWM/PID values */
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
static pidData_t            m_pid;

void serve_serial(void)
{
    int16_t ch = serial_read();
    uint8_t pwm_set_latch = 0, len = 0;
    if(ch >= '0' && ch <= '5')
    {
        int16_t new_target = ch - '0';
        if(new_target != m_pwm_target_value)
        {
            __disable_irq();
            pid_Reset_Integrator(&m_pid);
            m_pwm_target_value = new_target * 1000u;
            __enable_irq();
        }
    }
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
    EICRA = 0x02;
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

#define K_P 1.0
#define K_I 0.3
#define K_D 1.0

int main(void)
{
    /* Initialize everything before enabling output */
    init();
    serial_init();
    pid_Init(K_P * SCALING_FACTOR,
             K_I * SCALING_FACTOR,
             K_D * SCALING_FACTOR,
             &m_pid);
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
    pid_out = pid_Controller(m_pwm_target_value,
                             ad_mv,
                             &m_pid);
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
