//1
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include <Wire.h>

// Endereço I2C
const uint8_t I2C_ADDRESS = 9;

// Variáveis recebidas do Master
volatile float received_temp = 0.0;
volatile float received_dist = 0.0;
volatile uint8_t received_mov = 0;
volatile uint8_t received_incl = 0;
volatile uint8_t received_parada = 0;
volatile uint8_t received_temp_crit = 0;
volatile uint8_t received_madeira_eixo = 0;
volatile uint16_t received_blocks = 0;
volatile uint8_t new_data = 0;

// Variáveis de controle
volatile uint8_t parada_solicitada = 0;
uint8_t velocidade_motor_vertical = 127;
uint8_t velocidade_motor_horizontal = 127;

// Protótipos de função
void uart_init(unsigned int ubrr);
void uart_transmit(char data);
void uart_print(const char* str);
void uart_print_num(uint16_t num);
void i2c_init(uint8_t address);
void adc_init(void);
uint16_t adc_read(uint8_t channel);
void update_serial(void);
void send_command_to_master(void);
void receiveEvent(int howMany);
void requestEvent(void);

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

void i2c_init(uint8_t address) {
    Wire.begin(address);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);
}

void receiveEvent(int howMany) {
    if (howMany >= 9) {
        received_temp = (float)Wire.read();
        received_dist = (float)Wire.read();
        received_mov = Wire.read();
        received_incl = Wire.read();
        received_parada = Wire.read();
        received_temp_crit = Wire.read();
        received_madeira_eixo = Wire.read();
        received_blocks = Wire.read();
        received_blocks |= (Wire.read() << 8);
        new_data = 1;
    }
}

void requestEvent(void) {
    // Resposta ao master quando ele solicitar dados
    uint8_t response = parada_solicitada ? 0xFF : 0x01;
    Wire.write(response);
}

void send_command_to_master(void) {
    Wire.beginTransmission(8); // Endereço do master
    Wire.write(velocidade_motor_vertical);
    Wire.write(velocidade_motor_horizontal);
    byte error = Wire.endTransmission();
    
    if (error == 0) {
        uart_print("Comando enviado: V=");
        uart_print_num(velocidade_motor_vertical);
        uart_print(" H=");
        uart_print_num(velocidade_motor_horizontal);
        uart_print("\r\n");
    } else {
        uart_print("Erro envio comando: ");
        uart_print_num(error);
        uart_print("\r\n");
    }
}

void update_serial(void) {
    char buffer[16];
    
    uart_print("\n=== Dados do Chao de Fabrica ===\r\n");
    
    dtostrf(received_temp, 5, 2, buffer);
    uart_print("Temperatura: ");
    uart_print(buffer);
    uart_print(" C | ");
    uart_print(received_temp_crit ? "CRITICA" : "NORMAL");
    uart_print("\r\n");
    
    dtostrf(received_dist, 5, 2, buffer);
    uart_print("Nivel Tanque: ");
    uart_print(buffer);
    uart_print(" cm\r\n");
    
    uart_print("Movimento: ");
    uart_print(received_mov ? "DETECTADO" : "NAO DETECTADO");
    uart_print(" | Inclinacao: ");
    uart_print(received_incl ? "FORA EIXO" : "NORMAL");
    uart_print("\r\n");
    
    uart_print("Status Producao: ");
    uart_print(received_parada ? "PARADA" : "NORMAL");
    uart_print(" | Blocos: ");
    uart_print_num(received_blocks);
    uart_print("\r\n");
    
    uart_print("===============================\r\n");
}

int main(void) {
    // Inicializações
    uart_init(103);
    adc_init();
    i2c_init(I2C_ADDRESS);
    sei();
    
    uart_print("Slave (Supervisor) pronto\r\n");
    uart_print("Endereco I2C: ");
    uart_print_num(I2C_ADDRESS);
    uart_print("\r\n");
    
    unsigned long last_send = 0;
    
    while (1) {
        unsigned long current_time = millis();
        
        // Leitura dos potenciômetros a cada 100ms
        if (current_time - last_send >= 100) {
            velocidade_motor_vertical = adc_read(0) / 4; // A0 - Vertical
            velocidade_motor_horizontal = adc_read(1) / 4; // A1 - Horizontal
            send_command_to_master();
            last_send = current_time;
        }
        
        if (new_data) {
            update_serial();
            new_data = 0;
        }
    }
}