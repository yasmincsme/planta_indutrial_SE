#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include <Wire.h>
#include <U8glib.h>

// Configurações OLED
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_DEV_0 | U8G_I2C_OPT_NO_ACK | U8G_I2C_OPT_FAST);

// Endereços I2C
const uint8_t SLAVE_ADDRESS = 9;
const uint8_t MASTER_ADDRESS = 8;

// Definição de pinos para motores DC
#define ENABLE1_PIN PD5  // OC0B (PWM Motor Vertical)
#define ENABLE2_PIN PD6  // OC0A (PWM Motor Horizontal)
#define MOTOR1_PIN  PD4  // IN1 (Direção Motor Vertical)
#define MOTOR2_PIN  PD7  // IN3 (Direção Motor Horizontal)

// Outros pinos
#define LED_VERDE_PIN PB0
#define LED_VERMELHO_PIN PB1
#define BUZZER_PIN PD3
#define SERVO_PIN PB2
#define TEMP_CRITICA_MIN 10
#define TEMP_CRITICA_MAX 40

// Variáveis globais voláteis
volatile uint8_t mov_sensor = 0;
volatile uint8_t inclination_sensor = 0;
volatile uint8_t reading = 0;
volatile uint8_t send_data_flag = 0;
volatile uint8_t temperatura_critica = 0;
volatile uint8_t producao_parada = 0;
volatile uint8_t madeira_fora_eixo = 0;
volatile uint8_t novo_comando = 0;

// Variáveis de sensores e controle
float temperature = 0.0;
float distance = 0.0;
uint16_t block_count = 0;
uint8_t velocidade_motor_vertical = 0;
uint8_t velocidade_motor_horizontal = 0;
uint16_t rotacoes_motor_vertical = 0;
uint16_t rotacoes_motor_horizontal = 0;

// Protótipos de função
void uart_init(unsigned int ubrr);
void uart_transmit(char data);
void uart_print(const char* str);
void uart_print_num(uint16_t num);
void adc_init(void);
uint16_t adc_read(uint8_t channel);
void timer1_init(void);
void timer2_init(void);
void pwm_init(void);
void controlar_motores(void);
void send_temperature(float temp);
void send_distance(float dist);
void update_display(void);
void send_to_slave(void);
void receiveEvent(int howMany);
void requestEvent(void);
void show_startup_screen(void);
void verificar_temperatura(void);
void controlar_leds(void);
void verificar_inclinacao(void);
void cortar_madeira(void);

ISR(TIMER1_COMPA_vect) {
    reading = 1;
}

ISR(TIMER2_COMPA_vect) {
    send_data_flag = 1;
}

ISR(PCINT2_vect) {
    if (PIND & (1 << PD4)) {
        mov_sensor = 1;
    }
    if (PIND & (1 << PD5)) {
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
    OCR2A =  (15624 * 2) -1; // ~3ms
    TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20); // Prescaler 1024
    TIMSK2 |= (1 << OCIE2A);
}

void pwm_init(void) {
    // Configura Timer0 para Fast PWM nos motores DC
    TCCR0A |= (1 << COM0A1) | (1 << COM0B1); // PWM não inversor em OC0A e OC0B
    TCCR0A |= (1 << WGM00) | (1 << WGM01);   // Modo Fast PWM
    TCCR0B |= (1 << CS01);                   // Prescaler 8 (~7.8kHz)
    
    // Configura pinos dos motores como saída
    DDRD |= (1 << ENABLE1_PIN) | (1 << ENABLE2_PIN) | (1 << MOTOR1_PIN) | (1 << MOTOR2_PIN);
    
    // Configura direção dos motores (para frente)
    PORTD |= (1 << MOTOR1_PIN) | (1 << MOTOR2_PIN);
    
    // Configura PWM para servo (Timer2)
    TCCR2A |= (1 << COM2B1) | (1 << WGM21) | (1 << WGM20);
    DDRB |= (1 << SERVO_PIN);
}

void controlar_motores(void) {
    if (producao_parada || mov_sensor) {
        // Desliga motores
        OCR0A = 0; // Motor Horizontal
        OCR0B = 0; // Motor Vertical
    } else {
        // Aplica velocidades recebidas do supervisor
        OCR0A = velocidade_motor_horizontal; // Motor Horizontal
        OCR0B = velocidade_motor_vertical;   // Motor Vertical
        
        // Conta rotações (simulação)
        rotacoes_motor_vertical++;
        rotacoes_motor_horizontal++;
    }
}

void verificar_temperatura(void) {
    if (temperature < TEMP_CRITICA_MIN || temperature > TEMP_CRITICA_MAX) {
        if (!temperatura_critica) {
            temperatura_critica = 1;
            producao_parada = 1;
            uart_print("ALERTA: Temperatura Critica!\r\n");
            
            // Ativar buzzer
            PORTD |= (1 << BUZZER_PIN);
            _delay_ms(500);
            PORTD &= ~(1 << BUZZER_PIN);
        }
    } else {
        if (temperatura_critica) {
            temperatura_critica = 0;
            uart_print("AVISO: Temperatura normalizada\r\n");
        }
    }
}

void verificar_inclinacao(void) {
    if (inclination_sensor) {
        if (!madeira_fora_eixo) {
            madeira_fora_eixo = 1;
            producao_parada = 1;
            uart_print("ALERTA: Madeira fora do eixo!\r\n");
            
            // Posicionar servo para corrigir inclinação
            OCR2B = 25; // Posição corrigida
        }
    } else {
        if (madeira_fora_eixo) {
            madeira_fora_eixo = 0;
            uart_print("AVISO: Madeira posicionada corretamente\r\n");
            OCR2B = 10; // Posição neutra
        }
    }
}

void cortar_madeira(void) {
    // Verifica se completou 100 rotações (5cm)
    if (rotacoes_motor_vertical >= 100 && rotacoes_motor_horizontal >= 100) {
        rotacoes_motor_vertical = 0;
        rotacoes_motor_horizontal = 0;
        block_count++;
        
        // Verifica se completou um bloco (10cm x 25cm)
        // (Implementação adicional necessária para controle preciso)
    }
}

void controlar_leds(void) {
    if (producao_parada || temperatura_critica || madeira_fora_eixo) {
        PORTB &= ~(1 << LED_VERDE_PIN);  // Desliga verde
        PORTB |= (1 << LED_VERMELHO_PIN); // Liga vermelho
    } else {
        PORTB |= (1 << LED_VERDE_PIN);   // Liga verde
        PORTB &= ~(1 << LED_VERMELHO_PIN); // Desliga vermelho
    }
}

void receiveEvent(int howMany) {
    if (Wire.available() >= 2) {
        velocidade_motor_vertical = Wire.read();
        velocidade_motor_horizontal = Wire.read();
        novo_comando = 1;
    }
}

void requestEvent(void) {
    uint8_t response = producao_parada ? 0xFF : 0x01;
    Wire.write(response);
}

void update_display(void) {
    char buffer[16];
    char dezenas = (block_count / 10) % 10;
    char unidades = block_count % 10;

    u8g.firstPage();
    do {
        u8g.setFont(u8g_font_unifont);

        // Linha 1 - Status
        u8g.drawStr(0, 14, "STATUS:");
        if (producao_parada) {
            if (temperatura_critica) {
                u8g.drawStr(60, 14, "TEMP CRIT");
            } else if (madeira_fora_eixo) {
                u8g.drawStr(60, 14, "MAD. EIXO");
            } else {
                u8g.drawStr(60, 14, "PARADA");
            }
        } else {
            u8g.drawStr(60, 14, "NORMAL");
        }

        // Linha 2 - Motores
        u8g.drawStr(0, 30, "MOT V:");
        itoa(velocidade_motor_vertical, buffer, 10);
        u8g.drawStr(50, 30, buffer);
        
        u8g.drawStr(70, 30, "H:");
        itoa(velocidade_motor_horizontal, buffer, 10);
        u8g.drawStr(90, 30, buffer);

        // Linha 3 - Blocos
        u8g.drawStr(0, 48, "BLOCOS:");
        itoa(dezenas, buffer, 10);
        u8g.drawStr(70, 48, buffer);
        itoa(unidades, buffer, 10);
        u8g.drawStr(85, 48, buffer);

        // Linha 4 - Temperatura
        u8g.drawStr(0, 62, "TEMP:");
        dtostrf(temperature, 4, 1, buffer);
        u8g.drawStr(50, 62, buffer);

    } while (u8g.nextPage());
}

void send_to_slave(void) {
    uint8_t data[10];
    
    data[0] = (uint8_t)temperature;
    data[1] = (uint8_t)distance;
    data[2] = mov_sensor;
    data[3] = inclination_sensor;
    data[4] = producao_parada;
    data[5] = temperatura_critica;
    data[6] = madeira_fora_eixo;
    data[7] = block_count & 0xFF;
    data[8] = (block_count >> 8) & 0xFF;
    
    Wire.beginTransmission(SLAVE_ADDRESS);
    Wire.write(data, 9);
    byte error = Wire.endTransmission();
    
    if (error != 0) {
        uart_print("Erro I2C: ");
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
    // Inicializações
    uart_init(103);  // 9600 bps @ 16 MHz
    adc_init();
    timer1_init();
    timer2_init();
    pwm_init();
    Wire.begin(MASTER_ADDRESS);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);
    
    // Configuração de pinos
    DDRB &= ~((1 << DDD4) | (1 << DDD5)); // Entradas (sensores)
    DDRB |= (1 << LED_VERDE_PIN) | (1 << LED_VERMELHO_PIN) | (1 << SERVO_PIN); // Saídas
    DDRD |= (1 << ENABLE1_PIN) | (1 << ENABLE2_PIN) | (1 << MOTOR1_PIN) | (1 << MOTOR2_PIN) | (1 << BUZZER_PIN);
    
    // Pull-ups para sensores
    PORTB |= (1 << PORTB0) | (1 << PORTB1);
    
    // Inicialização
    PORTB &= ~((1 << LED_VERDE_PIN) | (1 << LED_VERMELHO_PIN));
    PORTD &= ~(1 << BUZZER_PIN);
    OCR2B = 10; // Posição neutra do servo
    
    // Interrupções
    PCICR |= (1 << PCIE2);
    PCMSK0 |= (1 << PCINT20) | (1 << PCINT21);
    sei();
    
    // Verifica OLED
    Wire.beginTransmission(0x3C);
    if (Wire.endTransmission() == 0) {
        u8g.setColorIndex(1);
        show_startup_screen();
        uart_print("OLED inicializado!\r\n");
    }
    
    uart_print("Master (Chao de Fabrica) pronto\r\n");
    
    while (1) {
        if (reading) {
            // Lê sensores
            temperature = (adc_read(0) * (5000.0 / 1024.0)) / 10.0;
            uint16_t raw_dist = adc_read(1);
            if (raw_dist > 3) distance = (6787.0 / (raw_dist - 3.0)) - 4.0;
            
            // Controles
            verificar_temperatura();
            verificar_inclinacao();
            controlar_motores();
            cortar_madeira();
            controlar_leds();
            update_display();
            
            // Log serial
            uart_print("T:");
            char buffer[16];
            dtostrf(temperature, 5, 2, buffer);
            uart_print(buffer);
            uart_print("C | D:");
            dtostrf(distance, 5, 2, buffer);
            uart_print(buffer);
            uart_print("cm | B:");
            uart_print_num(block_count);
            uart_print("\r\n");
            
            reading = 0;
        }
        
        if (send_data_flag) {
            send_to_slave();
            send_data_flag = 0;
        }
        
        if (novo_comando) {
            uart_print("Novo comando recebido - V:");
            uart_print_num(velocidade_motor_vertical);
            uart_print(" H:");
            uart_print_num(velocidade_motor_horizontal);
            uart_print("\r\n");
            novo_comando = 0;
        }
    }
}

