#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include <Wire.h>

// aqui eh o endereço I2C do disp arduino 1 (slave)
const uint8_t I2C_ADDRESS = 9;


volatile float received_temp = 0.0;
volatile float received_dist = 0.0;
volatile uint8_t received_mov = 0;
volatile uint8_t received_incl = 0;
volatile uint16_t received_blocks = 0;
volatile uint8_t new_data = 0;

void uart_init(unsigned int ubrr);
void uart_transmit(char data);
void uart_print(const char* str);
void uart_print_num(uint16_t num);
void i2c_init(uint8_t address);
void update_serial(void);

// rotina para receber o i2c
void receiveEvent(int howMany) {
    if (howMany >= 6) {
        received_temp = (float)Wire.read();
        received_dist = (float)Wire.read();
        received_mov = Wire.read();
        received_incl = Wire.read();
        received_blocks = Wire.read();
        received_blocks |= (Wire.read() << 8);
        new_data = 1;
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

void i2c_init(uint8_t address) {
    Wire.begin(address);
    Wire.onReceive(receiveEvent);
}

void update_serial(void) {
    char tempStr[16];
    char distStr[16];
    
    dtostrf(received_temp, 5, 2, tempStr);
    dtostrf(received_dist, 5, 2, distStr);
    
    uart_print("=== Dados do Chao de Fabrica ===\r\n");
    uart_print("Temperatura: ");
    uart_print(tempStr);
    uart_print(" C\r\n");
    
    uart_print("Distancia: ");
    uart_print(distStr);
    uart_print(" cm\r\n");
    
    uart_print("Movimento: ");
    uart_print(received_mov ? "SIM" : "NAO");
    uart_print("\r\n");
    
    uart_print("Inclinacao: ");
    uart_print(received_incl ? "SIM" : "NAO");
    uart_print("\r\n");
    
    uart_print("Blocos cortados: ");
    uart_print_num(received_blocks);
    uart_print("\r\n");
    uart_print("===============================\r\n");
}

int main(void) {
    // inicializações
    uart_init(103);  // 9600 bps @ 16 MHz
    i2c_init(I2C_ADDRESS);
    sei();
    
    uart_print("Slave (Supervisor) pronto\r\n");
    uart_print("Endereco I2C: ");
    uart_print_num(I2C_ADDRESS);
    uart_print("\r\n");
    
    while (1) {
        if (new_data) {
            update_serial();
            new_data = 0;
        }
    }
}