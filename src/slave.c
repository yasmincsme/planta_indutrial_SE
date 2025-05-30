#define F_CPU 16000000UL  

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include <Wire.h>

const uint8_t I2C_SLAVE_ADDRESS = 9;

volatile uint8_t master_producao_parada = 0;
volatile uint8_t master_temperatura_critica = 0;
volatile uint8_t master_madeira_fora_eixo = 0;
volatile uint8_t master_mov_sensor = 0;
volatile uint8_t master_nivel_oleo_critico = 0;
volatile uint16_t master_block_count = 0;
volatile uint8_t master_vel_vertical_rx = 0;
volatile uint8_t master_vel_horizontal_rx = 0;

volatile uint8_t new_data_from_master = 0;


volatile uint8_t parada_solicitada_supervisor = 0;  
uint8_t velocidade_motor_vertical_cmd = 127;
uint8_t velocidade_motor_horizontal_cmd = 127;

#define BOTAO_PARADA_SUPERVISOR_PIN PD2

volatile uint32_t g_ms_counter_slave = 0;

static volatile uint32_t last_button_press_time_supervisor_isr = 0;
const uint32_t DEBOUNCE_TIME_SUPERVISOR_MS = 250;

void uart_init(unsigned int ubrr);
void uart_transmit(char data);
void uart_print(const char* str);
void uart_print_num(uint16_t num);
void uart_print_float(float val, int precision);
void adc_init(void);
uint16_t adc_read(uint8_t channel);
void i2c_slave_init(uint8_t address);
void receiveEvent(int howMany);
void requestEvent(void);
void timer0_init_ms_counter_slave(void);
uint32_t get_ms_counter_slave(void);
void init_botao_parada_supervisor(void);
void update_serial_supervisor(void);

void timer0_init_ms_counter_slave(void) {
  TCCR0A |= (1 << WGM01);
  OCR0A = 249;
  TCCR0B |= (1 << CS01) | (1 << CS00);
  TIMSK0 |= (1 << OCIE0A);
}

ISR(TIMER0_COMPA_vect) {
  g_ms_counter_slave++;
}

uint32_t get_ms_counter_slave(void) {
  uint32_t ms_copy;
  cli();
  ms_copy = g_ms_counter_slave;
  sei();
  return ms_copy;
}

// coisas da uart 
void uart_init(unsigned int ubrr) {
  UBRR0H = (unsigned char)(ubrr >> 8);
  UBRR0L = (unsigned char)ubrr;
  UCSR0B = (1 << TXEN0);
  UCSR0C = (3 << UCSZ00);
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
  char buffer[7];
  itoa(num, buffer, 10);
  uart_print(buffer);
}
void uart_print_float(float val, int precision) {
  char buffer[16];
  dtostrf(val, 0, precision, buffer);
  uart_print(buffer);
}

void adc_init(void) {
  ADMUX = (1 << REFS0);
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}
uint16_t adc_read(uint8_t channel) {
  ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC));
  return ADC;
}

void i2c_slave_init(uint8_t address) {
  Wire.begin(address);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
}
void receiveEvent(int howMany) {
  if (howMany == 9) {
    master_temperatura_critica = Wire.read();
    master_madeira_fora_eixo = Wire.read();
    master_mov_sensor = Wire.read();
    master_nivel_oleo_critico = Wire.read();
    master_producao_parada = Wire.read();
    master_vel_vertical_rx = Wire.read();
    master_vel_horizontal_rx = Wire.read();
    uint8_t lsb_blocks = Wire.read();
    uint8_t msb_blocks = Wire.read();
    master_block_count = (msb_blocks << 8) | lsb_blocks;
    new_data_from_master = 1;
  } else {
    uart_print("Erro I2C Rx: #bytes != 9. Recebido: ");
    uart_print_num(howMany);
    uart_print("\r\n");
    while (Wire.available() > 0) {
      Wire.read();
    }
    new_data_from_master = 0;
  }
}
void requestEvent(void) {
  uint8_t data_to_send[3];
  data_to_send[0] = parada_solicitada_supervisor; // Envia o estado atual (0 ou 1)
  data_to_send[1] = velocidade_motor_vertical_cmd;
  data_to_send[2] = velocidade_motor_horizontal_cmd;
  Wire.write(data_to_send, 3);
}

void init_botao_parada_supervisor(void) {
  DDRD &= ~(1 << BOTAO_PARADA_SUPERVISOR_PIN);
  PORTD |= (1 << BOTAO_PARADA_SUPERVISOR_PIN);
  EICRA |= (1 << ISC01);
  EICRA &= ~(1 << ISC00);
  EIMSK |= (1 << INT0);
}

ISR(INT0_vect) {
  uint32_t current_time = get_ms_counter_slave();

  if (current_time - last_button_press_time_supervisor_isr > DEBOUNCE_TIME_SUPERVISOR_MS) {
    if (parada_solicitada_supervisor == 0) {
      parada_solicitada_supervisor = 1;
      uart_print("Botao SUPERVISOR: Solicitacao de PARADA enviada ao Master.\r\n");
    } else {
      parada_solicitada_supervisor = 0;
      uart_print("Botao SUPERVISOR: Solicitacao de REINICIO enviada ao Master.\r\n");
    }
    last_button_press_time_supervisor_isr = current_time;
  }
}

void update_serial_supervisor(void) {
  uart_print("\n=== Dados Recebidos do Chao de Fabrica (Master) ===\r\n");
  uart_print("Status Temp.: ");
  uart_print(master_temperatura_critica ? "CRITICA" : "NORMAL");
  uart_print("\r\n");
  uart_print("Inclinacao Madeira: ");
  uart_print(master_madeira_fora_eixo ? "FORA DO EIXO" : "NORMAL");
  uart_print("\r\n");
  uart_print("Movimento Detectado: ");
  uart_print(master_mov_sensor ? "SIM" : "NAO");
  uart_print("\r\n");
  uart_print("Status Nivel Oleo: ");
  uart_print(master_nivel_oleo_critico ? "CRITICO" : "OK");
  uart_print("\r\n");
  uart_print("Status Producao Master: ");
  uart_print(master_producao_parada ? "PARADA" : "NORMAL");
  uart_print("\r\n");
  uart_print("Velocidade Motor Vertical (RX Master): ");
  uart_print_num(master_vel_vertical_rx);
  uart_print("\r\n");
  uart_print("Velocidade Motor Horizontal (RX Master): ");
  uart_print_num(master_vel_horizontal_rx);
  uart_print("\r\n");
  uart_print("Blocos Cortados: ");
  uart_print_num(master_block_count);
  uart_print("\r\n");
  uart_print("===================================================\r\n");
  uart_print("--- Comandos do Supervisor para o Master ---\r\n");
  uart_print("Comando Supervisor (0=Reiniciar, 1=Parar): "); 
  uart_print_num(parada_solicitada_supervisor);
  uart_print("\r\n");
  uart_print("Velocidade Motor Vertical Cmd (de A0 do Supervisor): ");
  uart_print_num(velocidade_motor_vertical_cmd);
  uart_print("\r\n");
  uart_print("Velocidade Motor Horizontal Cmd (de A1 do Supervisor): ");
  uart_print_num(velocidade_motor_horizontal_cmd);
  uart_print("\r\n");
  uart_print("------------------------------------------\r\n");
}


int main(void) {
  uart_init(103);
  adc_init();
  timer0_init_ms_counter_slave();
  i2c_slave_init(I2C_SLAVE_ADDRESS);
  init_botao_parada_supervisor();

  sei();

  uart_print("Slave (Supervisor) pronto. Endereco I2C: ");
  uart_print_num(I2C_SLAVE_ADDRESS);
  uart_print("\r\n");

  uint32_t last_adc_read_time = 0;
  const uint32_t adc_read_interval = 100;
  uint32_t last_serial_print_time = 0;
  const uint32_t serial_print_interval = 3000;

  static uint8_t master_confirmou_parada_do_supervisor = 0;

  while (1) {
    uint32_t current_time_slave = get_ms_counter_slave();

    if (current_time_slave - last_adc_read_time >= adc_read_interval) {
      velocidade_motor_vertical_cmd = adc_read(0) / 4;
      velocidade_motor_horizontal_cmd = adc_read(1) / 4;
      last_adc_read_time = current_time_slave;
    }

    if (new_data_from_master) {
      new_data_from_master = 0;
    }

    if (current_time_slave - last_serial_print_time >= serial_print_interval) {
      update_serial_supervisor();
      last_serial_print_time = current_time_slave;
    }


    if (parada_solicitada_supervisor == 1 && master_producao_parada == 1) {
      if (!master_confirmou_parada_do_supervisor) {
        uart_print("CONFIRMACAO: Parada solicitada pelo Supervisor foi realizada pelo Master!\r\n");
        master_confirmou_parada_do_supervisor = 1; 
      }
    } else {
      master_confirmou_parada_do_supervisor = 0;
    }
  }
  return 0;
}