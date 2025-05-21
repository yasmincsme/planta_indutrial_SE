#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include <Wire.h>
#include <U8glib.h>

// config do oled 
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_DEV_0 | U8G_I2C_OPT_NO_ACK | U8G_I2C_OPT_FAST);

// endereço do slave p comunicação i2c
const uint8_t SLAVE_ADDRESS = 9;

// Variáveis globais voláteis
volatile uint8_t mov_sensor = 0;
volatile uint8_t inclination_sensor = 0;
volatile uint8_t reading = 0;
volatile uint8_t send_data_flag = 0;
volatile uint8_t production_stopped = 0;


// variaveis p sensores 
float temperature = 0.0;
float distance = 0.0;
uint16_t block_count = 0;

//  pinos dos leds e buzzer 
#define LED_VERDE_PIN PD7
#define LED_VERMELHO_PIN PD6
#define BUZZER_PIN PD5

// Protótipos de função
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
void check_temperature(void);
void control_production_leds(void);
void stop_production(void);
void resume_production(void);

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
    TCCR1B |= (1 << CS12) | (1 << CS10); // preescaler 1024
    TIMSK1 |= (1 << OCIE1A);
}

void timer2_init(void) {
    TCCR2A |= (1 << WGM21); // CTC
    OCR2A = 155; // ~3ms
    TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20); // prescaler 1024
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

void update_display(void) {
    char buffer[16];

    u8g.firstPage();
    do {
        u8g.setFont(u8g_font_unifont);

        // linha 1 - TEMP e MOV
        u8g.drawStr(0, 14, "TEMP:");
        dtostrf(temperature, 4, 1, buffer);
        strcat(buffer, " C");
        u8g.drawStr(50, 14, buffer);

        u8g.drawStr(0, 30, "MOV:");
        u8g.drawStr(50, 30, mov_sensor ? "SIM" : "NAO");

        // linha p separar
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
    
    data[0] = (uint8_t)temperature;
    data[1] = (uint8_t)distance;
    data[2] = mov_sensor;
    data[3] = inclination_sensor;
    data[4] = block_count & 0xFF;
    data[5] = (block_count >> 8) & 0xFF;
    data[6] = production_stopped;
    
    Wire.beginTransmission(SLAVE_ADDRESS);
    Wire.write(data, 7);
    Wire.endTransmission();
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

void check_temperature(void) {
    // p verificar se a temperatura está fora da faixa
    if (temperature < 10.0 || temperature > 30.0) {
        if (!production_stopped) {
            stop_production();
            uart_print("ALERTA: Temperatura Critica! ");
            send_temperature(temperature);
            
            // liga LED vermelho e buzzer
            PORTD |= (1 << LED_VERMELHO_PIN);
            PORTD |= (1 << BUZZER_PIN);
            
            // envia mensagem de parada por temperatura
            uint8_t alert_data[2] = {0xFF, 0x01}; // Código de alerta de temperatura
            Wire.beginTransmission(SLAVE_ADDRESS);
            Wire.write(alert_data, 2);
            Wire.endTransmission();
        }
    } else {
        // se a temperatura voltou ao normal e a parada foi por temperatura
        if (production_stopped) {
            resume_production();
            
            // desliga LED vermelho e buzzer
            PORTD &= ~(1 << LED_VERMELHO_PIN);
            PORTD &= ~(1 << BUZZER_PIN);
            
            uart_print("INFO: Temperatura Normalizada. ");
            send_temperature(temperature);
        }
    }
}

void control_production_leds(void) {
    if (production_stopped) {
        PORTD &= ~(1 << LED_VERDE_PIN);  // desliga LED verde
    } else {
        PORTD |= (1 << LED_VERDE_PIN);   // liga LED verde
    }
}

void stop_production(void) {
    production_stopped = 1;
    control_production_leds();
    // aaqui falta adicionar o codigo p paraada dos motores 
}

void resume_production(void) {
    production_stopped = 0;
    control_production_leds();
    // aqui tb falta adcionar o codigo p reinicio dos motores
}

int main(void) {
    // inicializações de tudooo
    uart_init(103);  // 9600 bps @ 16 MHz
    adc_init();
    timer1_init();
    timer2_init();
    Wire.begin();
    
    DDRB &= ~(1 << DDB0); // PB0 entrada (sensor movimento)
    DDRB &= ~(1 << DDB1); // PB1 entrada (sensor inclinação)
    DDRD |= (1 << LED_VERDE_PIN) | (1 << LED_VERMELHO_PIN) | (1 << BUZZER_PIN); // PD5, PD6 e PD7 como saída
    
    // no inicio ele desliga todos os LEDs e buzzer
    PORTD &= ~((1 << LED_VERDE_PIN) | (1 << LED_VERMELHO_PIN) | (1 << BUZZER_PIN));
    
    // pull-ups
    PORTB |= (1 << PORTB0);
    PORTB |= (1 << PORTB1);
    
    // interrupções
    PCICR |= (1 << PCIE0);
    PCMSK0 |= (1 << PCINT0) | (1 << PCINT1);
    sei();
    
    // verifica o oled 
    Wire.beginTransmission(0x3C);
    if (Wire.endTransmission() == 0) {
        u8g.setColorIndex(1);
        show_startup_screen();
        uart_print("OLED inicializado!\r\n");
    }
    
    uart_print("Master (Chao de Fabrica) pronto\r\n");
    
    // aqui inicia a producao e liga o led verde 
    resume_production();
    
    while (1) {
        if (reading) {
            // leitura dos sensores
            uint16_t raw_temp = adc_read(0); // A0: temperatura
            temperature = (raw_temp * (5000.0 / 1024.0)) / 10.0;
            
            uint16_t raw_dist = adc_read(1); // A1: distância IR
            if (raw_dist > 3) {
                distance = (6787.0 / (raw_dist - 3.0)) - 4.0;
            }
            
            // verifica temperatura e faz o controle dos leds 
            check_temperature();
            
            update_display();
            
            // envia para UART
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
        
       
        control_production_leds();
    }
}