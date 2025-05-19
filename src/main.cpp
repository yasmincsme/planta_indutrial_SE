#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>

volatile uint8_t mov_sensor;
volatile uint8_t inclination_sensor;
volatile uint8_t reading;

void uart_init(unsigned int ubrr) {
    UBRR0H = (unsigned char)(ubrr >> 8);
    UBRR0L = (unsigned char)ubrr;
    UCSR0B = (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void uart_transmit(char data) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}

void uart_print(const char* str) {
    while (*str) {
        uart_transmit(*str++);
    }
}

void uart_print_num(uint16_t num) {
    char buffer[6];
    itoa(num, buffer, 10);
    uart_print(buffer);
}

void adc_init(void) {
    ADMUX = (1 << REFS0); // AVcc
    ADCSRA = (1 << ADEN) |
             (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t adc_read(uint8_t channel) {
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F); // canal 0–7
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADC;
}

void timer1_init(void) {
    TCCR1B |= (1 << WGM12); // CTC
    OCR1A = 2 * 15624 - 1;         // 2s
    TCCR1B |= (1 << CS12) | (1 << CS10); // Prescaler 1024
    TIMSK1 |= (1 << OCIE1A);
}

ISR(TIMER1_COMPA_vect) {
    reading = 1;
}

void send_temperature(float temperature) {
    char buffer[16];
    dtostrf(temperature, 5, 2, buffer);
    uart_print("3: Temperatura: ");
    uart_print(buffer);
    uart_print(" C\r\n");
}

void send_distance(float distance) {
    char buffer[16];
    dtostrf(distance, 6, 2, buffer);
    uart_print("4: Distância: ");
    uart_print(buffer);
    uart_print(" cm\r\n");
}

ISR(PCINT0_vect) {
    if (PINB & (1 << PB0)) {
        mov_sensor = 1;
    }
    if (PINB & (1 << PB1)) {
        inclination_sensor = 1;
    }
}

int main(void) {
    uart_init(103);  // 9600 bps @ 16 MHz
    adc_init();
    timer1_init();

    DDRB &= ~(1 << DDB0); // PB0 entrada
    DDRB &= ~(1 << DDB1); // PB1 entrada
    DDRB |= (1 << DDB5);  // PB5 saída (LED)

    PORTB |= (1 << PORTB0);
    PORTB |= (1 << PORTB1);

    PCICR |= (1 << PCIE0);
    PCMSK0 |= (1 << PCINT0) | (1 << PCINT1);

    sei();

    float temperature;
    float distance;

    while (1) {
        if(reading) {
            if (mov_sensor) {
                uart_print("1: Movimento detectado\r\n");
                mov_sensor = 0;
            } else {
                uart_print("1: Movimento nao detectado\r\n");
            }
            if (inclination_sensor) {
                uart_print("2: Inclinado\r\n");
                inclination_sensor = 0;
            } else {
                uart_print("2: Estável\r\n");
            }

            uint16_t raw_temp = adc_read(1); // A1: temperatura
            temperature = (raw_temp * (5000.0 / 1024.0)) / 10.0;
            send_temperature(temperature);

            uint16_t raw_dist = adc_read(0); // A0: distância IR
            if (raw_dist > 3) {
                distance = (6787.0 / (raw_dist - 3.0)) - 4.0;
                send_distance(distance);
            }
            reading = 0;
            uart_print("-----------------------------\r\n");
        }   
    }
}
