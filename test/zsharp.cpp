
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>

void uart_init(unsigned int ubrr) {
    UBRR0H = (ubrr >> 8);
    UBRR0L = ubrr;
    UCSR0B = (1 << TXEN0);  // Habilita transmissor
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8N1
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

void adc_init() {
    ADMUX = (1 << REFS0); // Referência AVcc, entrada ADC0 (A0), resultado à direita
    ADCSRA = (1 << ADEN)  // Habilita ADC
           | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Prescaler 128 (125kHz @ 16MHz)
}

uint16_t adc_read() {
    ADCSRA |= (1 << ADSC); // Inicia conversão
    while (ADCSRA & (1 << ADSC)); // Aguarda término
    return ADC;
}

int main(void) {
    uart_init(103);    // 9600 bps @ 16 MHz
    adc_init();        // Inicializa ADC

    char buffer[10];
    uint16_t adc_value;
    float distancia;

    while (1) {
        adc_value = adc_read();

        // Fórmula de conversão para o Sharp GP2D12 (empírica)
        distancia = (6787.0 / (adc_value - 3.0)) - 4.0;

        // Conversão simples: mostra parte inteira da distância
        itoa((int)distancia, buffer, 10);
        uart_print(buffer);
        uart_print(" cm\r\n");

        _delay_ms(200);
    }
}
