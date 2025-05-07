#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>

volatile uint32_t ticks = 0;
volatile uint8_t flag_300ms = 0;

// --- Inicialização do ADC ---
void adc_init() {
    ADMUX = (1 << REFS0);  // Referência AVcc
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1); // Prescaler 64
}

uint16_t adc_read(uint8_t channel) {
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);  // Seleciona canal
    ADCSRA |= (1 << ADSC);                     // Inicia conversão
    while (ADCSRA & (1 << ADSC));              // Espera terminar
    return ADC;
}

// --- Inicialização da USART (Serial) ---
void uart_init() {
    uint16_t ubrr = 103;  // Baud rate 9600 para F_CPU = 16MHz
    UBRR0H = (ubrr >> 8);
    UBRR0L = ubrr;
    UCSR0B = (1 << TXEN0);              // Habilita transmissor
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);  // 8 bits, 1 stop, sem paridade
}

// Envia um caractere via Serial
void uart_putchar(char c) {
    while (!(UCSR0A & (1 << UDRE0)));  // Espera buffer vazio
    UDR0 = c;
}

// Envia uma string via Serial
void uart_puts(const char *s) {
    while (*s) {
        uart_putchar(*s++);
    }
}

// Envia um número inteiro (0–1023) como texto via Serial
void uart_print_uint16(uint16_t val) {
    char buffer[6];
    itoa(val, buffer, 10);  // Converte para string decimal
    uart_puts(buffer);
}

// Interrupção do Timer0: ocorre a cada 1ms
ISR(TIMER0_COMPA_vect) {
    ticks++;
    if (ticks >= 300) {
        ticks = 0;
        flag_300ms = 1;
    }
}

// Inicializa Timer0 para gerar interrupção a cada 1ms
void timer0_init() {
    TCCR0A = (1 << WGM01);               // modo CTC
    TCCR0B = (1 << CS01) | (1 << CS00);  // prescaler 64
    OCR0A = 249;                         // (16MHz / 64 / 1000Hz) - 1 = 249
    TIMSK0 = (1 << OCIE0A);              // habilita interrupção do Timer0
}

int main(void) {
    DDRB |= (1 << PB0) | (1 << PB1);  // D8 e D9 como saída

    adc_init();
    uart_init();
    timer0_init();
    sei();  // habilita interrupções globais

    while (1) {
        if (flag_300ms) {
            flag_300ms = 0;

            uint16_t pot_a2 = adc_read(2);  // A2 → LED D8
            uint16_t pot_a3 = adc_read(3);  // A3 → LED D9

            // Liga ou desliga LEDs com base no limiar
            if (pot_a2 > 150)
                PORTB |= (1 << PB0);
            else
                PORTB &= ~(1 << PB0);

            if (pot_a3 > 150)
                PORTB |= (1 << PB1);
            else
                PORTB &= ~(1 << PB1);

            // --- Impressão serial ---
            uart_puts("A2: ");
            uart_print_uint16(pot_a2);
            uart_puts(" | A3: ");
            uart_print_uint16(pot_a3);
            uart_puts("\r\n");
        }
    }
}
