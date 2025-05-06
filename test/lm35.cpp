#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>

void uart_init(void) {
    // Baud rate 9600 (UBRR = F_CPU / 16 / BAUD - 1)
    uint16_t ubrr = 103; // para 16 MHz e 9600 bps

    UBRR0H = (ubrr >> 8);
    UBRR0L = ubrr;
    UCSR0B = (1 << TXEN0);                    // Habilita transmissor
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);   // 8 bits de dados, 1 stop bit
}

void uart_send_char(char c) {
    while (!(UCSR0A & (1 << UDRE0)));  // Espera buffer estar livre
    UDR0 = c;
}

void uart_send_string(const char* str) {
    while (*str) {
        uart_send_char(*str++);
    }
}

void adc_init(void) {
    ADMUX = (1 << REFS0);               // Referência AVcc (5V), canal ADC0 (A0)
    ADCSRA = (1 << ADEN) |              // Habilita o ADC
             (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Prescaler 128
}

uint16_t adc_read(void) {
    ADCSRA |= (1 << ADSC);              // Inicia conversão
    while (ADCSRA & (1 << ADSC));       // Espera fim da conversão
    return ADC;
}

void send_temperature(float temperature) {
    char buffer[16];
    dtostrf(temperature, 5, 2, buffer);      // Converte float para string
    uart_send_string(buffer);                // Envia valor
    uart_send_string(" C\r\n");              // Envia 'C' + nova linha
}

int main(void) {
    uart_init();
    adc_init();

    while (1) {
        uint16_t reading = adc_read();

        // Conversão: tensão = leitura * (5000 mV / 1024)
        float voltage = reading * (5000.0 / 1024.0);

        // Temperatura = tensão / 10 mV/°C
        float temperature = voltage / 10.0;

        send_temperature(temperature);
        _delay_ms(1000);
    }
}
