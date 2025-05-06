#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>

volatile uint8_t led_state = 0;
volatile uint32_t last_interrupt_time = 0;
volatile uint32_t ticks = 0;

// Interrupção do Timer0: ocorre a cada 1ms
ISR(TIMER0_COMPA_vect) {
    ticks++;
}

// Interrupção externa INT0 (botão no pino D2)
ISR(INT0_vect) {
    if ((ticks - last_interrupt_time) > 200) {  // debounce de 200ms
        if (PIND & (1 << PD2)) {  // botão pressionado (nível alto)
            led_state = !led_state;
            if (led_state) {
                PORTD |= (1 << PD7);  // liga LED
            } else {
                PORTD &= ~(1 << PD7); // desliga LED
            }
            last_interrupt_time = ticks;
        }
    }
}

// Configuração do Timer0 para 1ms
void timer0_init() {
    TCCR0A = (1 << WGM01);               // modo CTC
    TCCR0B = (1 << CS01) | (1 << CS00);  // prescaler 64
    OCR0A = 249;                         // (16MHz / 64 / 1000) - 1 = 249
    TIMSK0 = (1 << OCIE0A);              // habilita interrupção do Timer0
}

int main(void) {
    // LED no PD7 como saída
    DDRD |= (1 << PD7);
    PORTD &= ~(1 << PD7);

    // Botão no PD2 como entrada
    DDRD &= ~(1 << PD2);
    PORTD &= ~(1 << PD2);  // sem pull-up interno

    // Configura INT0 para borda de subida
    EICRA |= (1 << ISC01) | (1 << ISC00);
    EIMSK |= (1 << INT0);

    timer0_init();  // inicializa temporizador
    sei();          // habilita interrupções globais

    while (1) {
        // loop principal vazio
    }
}
