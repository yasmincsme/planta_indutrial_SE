#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include <Wire.h>
#include <U8glib.h>

U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_DEV_0 | U8G_I2C_OPT_NO_ACK | U8G_I2C_OPT_FAST);

// endereço I2C do Slave
const uint8_t SLAVE_ADDRESS = 9;

#define LED_VERDE_PIN PD7
#define LED_VERMELHO_PIN PD6
#define BUZZER_PIN PD5
#define TEMP_CRITICA_MIN 10
#define TEMP_CRITICA_MAX 40

volatile uint8_t mov_sensor = 0;
volatile uint8_t inclination_sensor = 0;
volatile uint8_t reading = 0;
volatile uint8_t send_data_flag = 0;
volatile uint8_t producao_parada = 0;
volatile uint8_t temperatura_critica = 0;


float temperature = 0.0;
float distance = 0.0;
uint16_t block_count = 0;

void uart_init(unsigned int ubrr);
void uart_transmit(char data);
void uart_print(const char* str);
void uart_print_num(uint16_t num);
void adc_init(void);
uint16_t adc_read(uint8_t channel);
void timer1_init(void);
void timer2_init(void);
void send_temperature(float temp);
void send_distance(float dist);
void update_display(void);
void send_to_slave(void);
void show_startup_screen(void);
void verificar_temperatura(void);
void controlar_leds(void);
void parar_producao(void);
void reiniciar_producao(void);

ISR(TIMER1_COMPA_vect) {
    reading = 1;
}

ISR(TIMER2_COMPA_vect) {
    send_data_flag = 1;
}

ISR(PCINT0_vect) {
    if (PINB & (1 << PB0)) {
        mov_sensor = 1;
    }
    if (PINB & (1 << PB1)) {
        inclination_sensor = 1;
    }
}

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
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t adc_read(uint8_t channel) {
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADC;
}

void timer1_init(void) {
    TCCR1B |= (1 << WGM12); // CTC
    OCR1A = 15624; // 1s com prescaler 1024
    TCCR1B |= (1 << CS12) | (1 << CS10); // Prescaler 1024
    TIMSK1 |= (1 << OCIE1A);
}

void timer2_init(void) {
    TCCR2A |= (1 << WGM21); // CTC
    OCR2A = 155; // ~3ms
    TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20); // Prescaler 1024
    TIMSK2 |= (1 << OCIE2A);
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
    uart_print("4: Distancia: ");
    uart_print(buffer);
    uart_print(" cm\r\n");
}

void verificar_temperatura(void) {
    if (temperature < TEMP_CRITICA_MIN || temperature > TEMP_CRITICA_MAX) {
        if (!temperatura_critica) {
            temperatura_critica = 1;
            parar_producao();
            uart_print("ALERTA: Temperatura Critica!\r\n");
            
            // Ativar buzzer
            PORTD |= (1 << BUZZER_PIN);
            _delay_ms(500);
            PORTD &= ~(1 << BUZZER_PIN);
        }
    } else {
        if (temperatura_critica) {
            temperatura_critica = 0;
            reiniciar_producao();
        }
    }
}

void parar_producao(void) {
    producao_parada = 1;
    // Enviar comando de parada para Slave
    uint8_t data[2] = {0xFF, 0x00}; // Código de parada
    Wire.beginTransmission(SLAVE_ADDRESS);
    Wire.write(data, 2);
    Wire.endTransmission();
}

void reiniciar_producao(void) {
    producao_parada = 0;
    // Enviar comando de reinicio para Slave
    uint8_t data[2] = {0xFF, 0x01}; // Código de reinicio
    Wire.beginTransmission(SLAVE_ADDRESS);
    Wire.write(data, 2);
    Wire.endTransmission();
}

void controlar_leds(void) {
    if (producao_parada) {
        PORTD &= ~(1 << LED_VERDE_PIN);  // Desliga verde
        PORTD |= (1 << LED_VERMELHO_PIN); // Liga vermelho
    } else {
        PORTD |= (1 << LED_VERDE_PIN);   // Liga verde
        PORTD &= ~(1 << LED_VERMELHO_PIN); // Desliga vermelho
    }
    
    if (temperatura_critica) {
        // piscar vermelho rápido em caso de temperatura crítica
        if (reading) {
            PORTD ^= (1 << LED_VERMELHO_PIN);
        }
    }
}

void update_display(void) {
    char buffer[16];

    u8g.firstPage();
    do {
        u8g.setFont(u8g_font_unifont);

        // linha 1 - TEMP e STATUS
        u8g.drawStr(0, 14, "TEMP:");
        dtostrf(temperature, 4, 1, buffer);
        strcat(buffer, " C");
        u8g.drawStr(50, 14, buffer);
        
        u8g.drawStr(0, 30, "STATUS:");
        if (producao_parada) {
            if (temperatura_critica) {
                u8g.drawStr(60, 30, "TEMP CRIT!");
            } else {
                u8g.drawStr(60, 30, "PARADA");
            }
        } else {
            u8g.drawStr(60, 30, "NORMAL");
        }

        // linha de separação
        u8g.drawHLine(0, 36, 128);

        // linha 3 - INCL e BLOCOS
        u8g.drawStr(0, 48, "INCL:");
        u8g.drawStr(50, 48, inclination_sensor ? "SIM" : "NAO");

        u8g.drawStr(0, 62, "BLOCOS:");
        itoa(block_count, buffer, 10);
        u8g.drawStr(70, 62, buffer);

    } while (u8g.nextPage());
}

void send_to_slave(void) {
    uint8_t data[10];
    
    // monta pacote de dados
    data[0] = (uint8_t)temperature;
    data[1] = (uint8_t)distance;
    data[2] = mov_sensor;
    data[3] = inclination_sensor;
    data[4] = producao_parada;
    data[5] = temperatura_critica;
    data[6] = block_count & 0xFF;
    data[7] = (block_count >> 8) & 0xFF;
    
    Wire.beginTransmission(SLAVE_ADDRESS);
    Wire.write(data, 8);
    byte error = Wire.endTransmission();
    
    if (error != 0) {
        uart_print("Erro na comunicacao I2C: ");
        uart_print_num(error);
        uart_print("\r\n");
    }
}

void show_startup_screen(void) {
    u8g.firstPage();
    do {
        u8g.setFont(u8g_font_unifont);
        u8g.drawStr(0, 20, "Sistema Iniciado");
        u8g.drawStr(0, 40, "Chao de Fabrica");
        u8g.drawStr(0, 60, "Master I2C");
    } while (u8g.nextPage());
}

int main(void) {
    uart_init(103);  // 9600 bps @ 16 MHz
    adc_init();
    timer1_init();
    timer2_init();
    Wire.begin();

    DDRB &= ~(1 << DDB0); // PB0 entrada (sensor movimento)
    DDRB &= ~(1 << DDB4); // PB1 entrada (sensor inclinação)
    DDRD |= (1 << LED_VERDE_PIN) | (1 << LED_VERMELHO_PIN) | (1 << BUZZER_PIN); // Saídas
    
    // pull-ups
    PORTB |= (1 << PORTB0) | (1 << PORTB1);
    
    // desliga buzzer e LEDs inicialmente
    PORTD &= ~((1 << LED_VERDE_PIN) | (1 << LED_VERMELHO_PIN) | (1 << BUZZER_PIN));
    
    // interrupções
    PCICR |= (1 << PCIE0);
    PCMSK0 |= (1 << PCINT0) | (1 << PCINT1);
    sei();
    
    // verifica OLED
    Wire.beginTransmission(0x3C);
    if (Wire.endTransmission() == 0) {
        u8g.setColorIndex(1);
        show_startup_screen();
        uart_print("OLED inicializado!\r\n");
    }
    
    uart_print("Master (Chao de Fabrica) pronto\r\n");
    
    while (1) {
        if (reading) {

            uint16_t raw_temp = adc_read(0); // A0: temperatura
            temperature = (raw_temp * (5000.0 / 1024.0)) / 10.0;
            
            uint16_t raw_dist = adc_read(1); // A1: distância IR
            if (raw_dist > 3) {
                distance = (6787.0 / (raw_dist - 3.0)) - 4.0;
            }
            
    
            verificar_temperatura();
                 
            controlar_leds();       
            update_display();
            
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
                uart_print("2: Estavel\r\n");
            }
            
            send_temperature(temperature);
            send_distance(distance);
            uart_print("-----------------------------\r\n");
            
            reading = 0;
        }
        
        if (send_data_flag) {
            send_to_slave();
            send_data_flag = 0;
        }
    }
}