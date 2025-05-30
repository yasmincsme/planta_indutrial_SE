#define F_CPU 16000000UL 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdio.h>
#include <Wire.h>
#include <U8glib.h>

U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_DEV_0 | U8G_I2C_OPT_NO_ACK | U8G_I2C_OPT_FAST);

const uint8_t SUPERVISOR_ADDRESS = 9; 

#define ENABLE_MOTOR_VERTICAL_PIN PD5    
#define MOTOR_VERTICAL_DIR1_PIN PD4      

#define ENABLE_MOTOR_HORIZONTAL_PIN PD6 // timer0
#define MOTOR_HORIZONTAL_DIR1_PIN PD7  


#define SERVO_INCLINACAO_PIN PB1   // timer 1 

#define BUZZER_PIN PC3         //  A5 

#define BOTAO_PARADA_CHAO_PIN PD2  

#define MOV_SENSOR_PIN PB4            // 12
#define INCLINATION_SENSOR_PIN PB3    // D11 

#define NIVEL_OLEO_SENSOR_PIN PC4     //  A4

#define LED_VERMELHO_PIN PB0          // D8 
#define LED_VERDE_PIN PB2             // D10 no Nano
#define LED_INCLINACAO_PIN PD3

#define TEMP_CRITICA_MIN 10
#define TEMP_CRITICA_MAX 40
#define DOUBLE_CLICK_TIME 500


volatile uint8_t mov_sensor_status = 0;           
volatile uint8_t inclination_sensor_status = 0;   
volatile uint8_t nivel_oleo_critico_status = 0;

volatile uint8_t temperatura_critica_status = 0;
volatile uint8_t producao_parada_status = 0;
volatile uint8_t madeira_fora_eixo_status = 0; 

volatile uint8_t supervisor_parada_cmd = 0;
volatile uint8_t cmd_velocidade_motor_vertical = 0;
volatile uint8_t cmd_velocidade_motor_horizontal = 0;
volatile uint8_t novos_comandos_recebidos = 0;

volatile uint8_t buzzer_request = 0; 
volatile uint8_t buzzer_beeping_state = 0;
volatile uint32_t buzzer_beep_start_time_ms = 0;
const uint32_t BUZZER_BEEP_DURATION_MS = 200;


float current_temperature = 0.0;
float current_distance = 0.0; 
uint16_t block_count = 0;

volatile uint16_t rotacoes_motor_vertical_acumuladas = 0;
volatile uint16_t rotacoes_motor_horizontal_acumuladas = 0;
const uint16_t ROTATIONS_FOR_5CM = 100;
const uint8_t CM_PER_BLOCK_VERTICAL = 10;
const uint8_t CM_PER_BLOCK_HORIZONTAL = 25;
const uint16_t ROTATIONS_NEEDED_VERTICAL = (CM_PER_BLOCK_VERTICAL / 5) * ROTATIONS_FOR_5CM;  
const uint16_t ROTATIONS_NEEDED_HORIZONTAL = (CM_PER_BLOCK_HORIZONTAL / 5) * ROTATIONS_FOR_5CM;
uint8_t vertical_cut_done = 0;
uint8_t horizontal_cut_done = 0;



volatile uint8_t botao_parada_master_pressionado = 0;
volatile uint32_t ultimo_tempo_clique_master = 0;
volatile uint8_t clique_duplo_master = 0;

volatile uint32_t g_ms_counter = 0;


void uart_init(unsigned int ubrr);
void uart_transmit(char data);
void uart_print(const char* str);
void uart_print_num(uint16_t num);
void uart_print_float(float val, int precision);

void adc_init(void);
uint16_t adc_read(uint8_t channel);

void timer2_init_ms_counter(void);
void pwm_init_l298n_and_servo(void);

void controlar_motores_dc(void);
void controlar_servo_inclinacao(void); 
void controlar_leds(void);
void handle_buzzer(void); 

void verificar_condicoes_producao(void);
void cortar_madeira(void);

void init_botao_parada_chao(void);
void tratar_botao_parada_chao(void);

void update_display_chao(void);
void show_startup_screen_chao(void);

void send_status_to_supervisor(void);
void request_commands_from_supervisor(void);
uint8_t check_oled_presence(uint8_t addr);


uint32_t get_ms_counter(void) {
    uint32_t ms_copy;
    cli();
    ms_copy = g_ms_counter;
    sei();
    return ms_copy;
}

ISR(TIMER2_COMPA_vect) {
    g_ms_counter++;
}


ISR(PCINT0_vect) {

    if (PINB & (1 << MOV_SENSOR_PIN)) { 
        mov_sensor_status = 1;
    } else { 
        mov_sensor_status = 0;
    }

    if (PINB & (1 << INCLINATION_SENSOR_PIN)) {
        inclination_sensor_status = 1;
    } else { 
        inclination_sensor_status = 0;
    }
}

ISR(INT0_vect) { 
    botao_parada_master_pressionado = 1;
}


#define ADC_SAMPLES_TO_AVERAGE 16 

uint16_t adc_read_averaged(uint8_t channel, uint8_t num_samples) {
    uint32_t sum = 0;
    

    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F); 

    for (volatile uint8_t i=0; i<10; ++i); 

    for (uint8_t i = 0; i < num_samples; ++i) {
        ADCSRA |= (1 << ADSC);        
        while (ADCSRA & (1 << ADSC)); 
        sum += ADC;                   
    }
    return (uint16_t)(sum / num_samples); 
}

void uart_init(unsigned int ubrr) {
    UBRR0H = (unsigned char)(ubrr >> 8);
    UBRR0L = (unsigned char)ubrr;
    UCSR0B = (1 << TXEN0) | (1 << RXEN0); 
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); 
}
void uart_transmit(char data) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}
void uart_print(const char* str) {
    while (*str) uart_transmit(*str++);
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
    ADCSRA |= (1 << ADSC); // inicia conversão
    while (ADCSRA & (1 << ADSC)); // pequena espera p completar 
    return ADC;
}

void timer2_init_ms_counter(void) {
    TCCR2A = (1 << WGM21); // modo CTC
    OCR2A = 249;           // para 1ms: (16MHz / 64 / 1000Hz) - 1 = 249
    TCCR2B = (1 << CS22);  // prescaler 64 para Timer2
    TIMSK2 = (1 << OCIE2A);// habilita interrupção de comparação A do Timer2
}

void pwm_init_l298n_and_servo(void) {
    // timer0 
    DDRD |= (1 << ENABLE_MOTOR_VERTICAL_PIN) | (1 << ENABLE_MOTOR_HORIZONTAL_PIN); 
    DDRD |= (1 << MOTOR_VERTICAL_DIR1_PIN) | (1 << MOTOR_HORIZONTAL_DIR1_PIN);     

    PORTD |= (1 << MOTOR_VERTICAL_DIR1_PIN); 
    PORTD |= (1 << MOTOR_HORIZONTAL_DIR1_PIN);

    TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << WGM01) | (1 << WGM00); 
    TCCR0B = (1 << CS01); 
    OCR0A = 0; // PD6
    OCR0B = 0; // PD5

    DDRB |= (1 << SERVO_INCLINACAO_PIN); // PB1

    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);    
    ICR1 = 39999;
    OCR1A = 3000; 
}

void controlar_led_inclinacao(void) {
    if (madeira_fora_eixo_status) { 
        PORTB |= (1 << LED_INCLINACAO_PIN);  
    } else { 
        PORTB &= ~(1 << LED_INCLINACAO_PIN); 
    }
}

void init_botao_parada_chao(void) {
    DDRD &= ~(1 << BOTAO_PARADA_CHAO_PIN); 
    PORTD |= (1 << BOTAO_PARADA_CHAO_PIN); 
    EICRA |= (1 << ISC01); // interrupção INT0 na borda de descida
    EIMSK |= (1 << INT0);  
}

void handle_buzzer(void) {
    if (buzzer_request && !buzzer_beeping_state) {
        PORTC |= (1 << BUZZER_PIN);
        buzzer_beep_start_time_ms = get_ms_counter();
        buzzer_beeping_state = 1;
        buzzer_request = 0; 
    }

    if (buzzer_beeping_state) {
        if (get_ms_counter() - buzzer_beep_start_time_ms >= BUZZER_BEEP_DURATION_MS) {
            PORTC &= ~(1 << BUZZER_PIN);
            buzzer_beeping_state = 0;
        }
    }
}



void controlar_motores_dc(void) {
    if (producao_parada_status || mov_sensor_status ) { 
        OCR0A = 0; 
        OCR0B = 0; 
    } else {
        PORTD |= (1 << MOTOR_VERTICAL_DIR1_PIN);
        PORTD |= (1 << MOTOR_HORIZONTAL_DIR1_PIN);

        OCR0B = cmd_velocidade_motor_vertical;   
        OCR0A = cmd_velocidade_motor_horizontal;


        if (cmd_velocidade_motor_vertical > 0) {
            if (vertical_cut_done == 0) rotacoes_motor_vertical_acumuladas += (cmd_velocidade_motor_vertical / 50); // Ajustar este fator
        }
        if (cmd_velocidade_motor_horizontal > 0) {
            if (horizontal_cut_done == 0) rotacoes_motor_horizontal_acumuladas += (cmd_velocidade_motor_horizontal / 50); // Ajustar este fator
        }
    }
}

void controlar_servo_inclinacao(void) {
    if (madeira_fora_eixo_status) {
        OCR1A = 2500; 
        PORTD |= (1 << LED_INCLINACAO_PIN);
    } else {
        OCR1A = 3500; 
        PORTD &= ~(1 << LED_INCLINACAO_PIN);
    }
}


void controlar_leds(void) {
    if (producao_parada_status || temperatura_critica_status || madeira_fora_eixo_status || mov_sensor_status || nivel_oleo_critico_status) {
        PORTB &= ~(1 << LED_VERDE_PIN); 
        PORTB |= (1 << LED_VERMELHO_PIN); 
    } else {
        PORTB |= (1 << LED_VERDE_PIN);   
        PORTB &= ~(1 << LED_VERMELHO_PIN); 
    }
}

void verificar_condicoes_producao(void) {

    if (current_temperature < TEMP_CRITICA_MIN || current_temperature > TEMP_CRITICA_MAX) {
        if (!temperatura_critica_status) {
            temperatura_critica_status = 1;
            producao_parada_status = 1;
            uart_print("ALERTA: Temperatura Critica! Producao parada.\r\n");
            buzzer_request = 1; 
        }
    } else {
        if (temperatura_critica_status) {
            temperatura_critica_status = 0;
            uart_print("AVISO: Temperatura normalizada.\r\n");

            if (!madeira_fora_eixo_status && 
                !nivel_oleo_critico_status && 
                !supervisor_parada_cmd &&  
                !mov_sensor_status) {      
                
                producao_parada_status = 0; 
                uart_print("INFO: Temperatura OK e sem outras falhas. Producao pode ser retomada.\r\n");
            } else {
                uart_print("INFO: Temperatura OK, mas outra condicao critica/parada impede retomada automatica.\r\n");
            }
        }
    }

    if (inclination_sensor_status) { 
        if (!madeira_fora_eixo_status) {
            madeira_fora_eixo_status = 1;
            producao_parada_status = 1;
            uart_print("ALERTA: Madeira fora do eixo! Producao parada.\r\n");
        }
    } else { 
        if (madeira_fora_eixo_status) {
            madeira_fora_eixo_status = 0;
            producao_parada_status = 0;
            uart_print("AVISO: Madeira posicionada corretamente.\r\n");
        }
    }
    controlar_servo_inclinacao(); 


    uint8_t nivel_lido_critico = (PINC & (1 << NIVEL_OLEO_SENSOR_PIN)) ? 0 : 1; 
    if (nivel_lido_critico) {
        if (!nivel_oleo_critico_status) {
            nivel_oleo_critico_status = 1;
            producao_parada_status = 1;
            uart_print("ALERTA: Nivel de oleo critico! Producao parada.\r\n");
            buzzer_request = 1; 
        }
    } else {
        if (nivel_oleo_critico_status) {
            nivel_oleo_critico_status = 0;
            uart_print("AVISO: Nivel de oleo normalizado.\r\n");
        }
    }
    if (mov_sensor_status) {
         uart_print("AVISO: Presenca humana detectada! Motores parados.\r\n"); 
    }
}



void cortar_madeira(void) {
    if (!producao_parada_status && !mov_sensor_status) {
        if (!vertical_cut_done && rotacoes_motor_vertical_acumuladas >= ROTATIONS_NEEDED_VERTICAL) {
            vertical_cut_done = 1;
            rotacoes_motor_vertical_acumuladas = 0;
        }

        if (!horizontal_cut_done && rotacoes_motor_horizontal_acumuladas >= ROTATIONS_NEEDED_HORIZONTAL) {
            horizontal_cut_done = 1;
            rotacoes_motor_horizontal_acumuladas = 0; 
        }

        if (vertical_cut_done && horizontal_cut_done) {
            block_count++;
            vertical_cut_done = 0;   
            horizontal_cut_done = 0; 
            //uart_print("INFO: Bloco de madeira cortado! Total: ");
            //uart_print_num(block_count);
            //uart_print("\r\n");
        }
    } else {
    }
}


void tratar_botao_parada_chao(void) {
    if (botao_parada_master_pressionado) {
        uint32_t tempo_atual = get_ms_counter();
        botao_parada_master_pressionado = 0; 

        if (tempo_atual - ultimo_tempo_clique_master < 50) return;

        if (ultimo_tempo_clique_master != 0 && (tempo_atual - ultimo_tempo_clique_master < DOUBLE_CLICK_TIME)) {
            clique_duplo_master = 1;
        } else {
            clique_duplo_master = 0;
        }
        ultimo_tempo_clique_master = tempo_atual;
        if (clique_duplo_master) { 
            producao_parada_status = 0;
            supervisor_parada_cmd = 0; 
            uart_print("Producao TENTANDO REINICIAR (botao local duplo clique)!\r\n");
            
            verificar_condicoes_producao(); 

            if (temperatura_critica_status || madeira_fora_eixo_status || nivel_oleo_critico_status || mov_sensor_status) {
                producao_parada_status = 1; 
                uart_print("REINICIO FALHOU: Condicao critica persiste.\r\n");
            } else {
                uart_print("Producao REINICIADA (botao local).\r\n");
                rotacoes_motor_vertical_acumuladas = 0;
                rotacoes_motor_horizontal_acumuladas = 0;
                vertical_cut_done = 0;
                horizontal_cut_done = 0;
            }
            clique_duplo_master = 0;
        } else { 
            if (!producao_parada_status) {
                 producao_parada_status = 1;
                 uart_print("Parada de emergencia LOCAL acionada (clique simples)!\r\n");
            }
        }
    }
}


void update_display_chao(void) {
    char buffer[30];
    char temp_str[10];
    int x;

    u8g.firstPage();
    do {
        u8g.setFont(u8g_font_8x13B);

        const char* titulo = "Chao de Fabrica";
        x = (128 - u8g.getStrWidth(titulo)) / 2;
        u8g.drawStr(x, 12, titulo);

        dtostrf(current_temperature, 4, 1, temp_str);
        sprintf(buffer, "Temp: %sC", temp_str);
        x = (128 - u8g.getStrWidth(buffer)) / 2;
        u8g.drawStr(x, 26, buffer);

        u8g.drawHLine(0, 34, 128);

        sprintf(buffer, "Blocos: %u", block_count);
        x = (128 - u8g.getStrWidth(buffer)) / 2;
        u8g.drawStr(x, 46, buffer);
        
        if (producao_parada_status) {
            if (supervisor_parada_cmd) sprintf(buffer, "Status: PARADA(SUP)");
            else if (temperatura_critica_status) sprintf(buffer, "Status: TEMP CRIT");
            else if (madeira_fora_eixo_status) sprintf(buffer, "Status: FORA EIXO");
            else if (nivel_oleo_critico_status) sprintf(buffer, "Status: OLEO BAIXO");
            else if (mov_sensor_status) sprintf(buffer, "Status: PRESENCA");
            else sprintf(buffer, "Status: PARADA");
        } else {
            sprintf(buffer, "Status: PRODUCAO");
        }
        x = (128 - u8g.getStrWidth(buffer)) / 2;
        u8g.drawStr(x, 60, buffer);

    } while (u8g.nextPage());
}



void show_startup_screen_chao(void) {
    u8g.firstPage();
    do {
        u8g.setFont(u8g_font_unifont);
        u8g.drawStr(0, 20, "Sistema Iniciado");
        u8g.drawStr(0, 40, "CHAO DE FABRICA");
        u8g.drawHLine(0, 60, 120);
    } while (u8g.nextPage());
}


void send_status_to_supervisor(void) {
    uint8_t data_packet[11];
    data_packet[0] = producao_parada_status;
    data_packet[1] = temperatura_critica_status;
    data_packet[2] = madeira_fora_eixo_status;
    data_packet[3] = mov_sensor_status;
    data_packet[4] = nivel_oleo_critico_status;
    data_packet[5] = block_count & 0xFF;        
    data_packet[6] = (block_count >> 8) & 0xFF;  
    data_packet[7] = cmd_velocidade_motor_vertical; 
    data_packet[8] = cmd_velocidade_motor_horizontal; 
    data_packet[9] = producao_parada_status;
    uint8_t final_data_packet[9];
    final_data_packet[0] = temperatura_critica_status;
    final_data_packet[1] = madeira_fora_eixo_status;
    final_data_packet[2] = mov_sensor_status;
    final_data_packet[3] = nivel_oleo_critico_status;
    final_data_packet[4] = producao_parada_status; 
    final_data_packet[5] = cmd_velocidade_motor_vertical;
    final_data_packet[6] = cmd_velocidade_motor_horizontal;
    final_data_packet[7] = block_count & 0xFF;
    final_data_packet[8] = (block_count >> 8) & 0xFF;


    Wire.beginTransmission(SUPERVISOR_ADDRESS);
    Wire.write(final_data_packet, sizeof(final_data_packet)); 
    byte error = Wire.endTransmission();

    if (error != 0) {
        uart_print("Erro I2C Tx Status: ");
        uart_print_num(error);
        uart_print("\r\n");
    }
}

void request_commands_from_supervisor(void) {
    uint8_t bytes_count = Wire.requestFrom(SUPERVISOR_ADDRESS, (uint8_t)3); 

    if (bytes_count == 3) {
        uint8_t parada_cmd_recebida_do_supervisor = Wire.read();
        uint8_t parada_cmd_anterior_interna = supervisor_parada_cmd; 

        supervisor_parada_cmd = parada_cmd_recebida_do_supervisor; 

        cmd_velocidade_motor_vertical = Wire.read();
        cmd_velocidade_motor_horizontal = Wire.read();
        novos_comandos_recebidos = 1; 


        if (supervisor_parada_cmd == 1 && parada_cmd_anterior_interna == 0) {
            producao_parada_status = 1; // força a parada da produção
            uart_print("PARADA SOLICITADA PELO SUPERVISOR EFETUADA!\r\n");
        } else if (supervisor_parada_cmd == 0 && parada_cmd_anterior_interna == 1) {
            uart_print("SUPERVISOR SOLICITOU REINICIO DE PRODUCAO. Tentando...\r\n");
            
            producao_parada_status = 0;
            verificar_condicoes_producao(); 

            if (producao_parada_status == 0) { 
                uart_print("Producao REINICIADA (comando do Supervisor).\r\n");
                rotacoes_motor_vertical_acumuladas = 0;
                rotacoes_motor_horizontal_acumuladas = 0;
                vertical_cut_done = 0;
                horizontal_cut_done = 0;
            } else {
                uart_print("REINICIO SOLICITADO PELO SUPERVISOR FALHOU: Condicao critica local persiste.\r\n");
            }
        }

    } else {
        // uart_print("erroooo"); 
    }
}

uint8_t check_oled_presence(uint8_t addr) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
        return 1;
    }
    return 0;
}

int main(void) {
    uart_init(103); 
    adc_init();
    timer2_init_ms_counter();
    pwm_init_l298n_and_servo();
    Wire.begin(); 
    init_botao_parada_chao();
    
    DDRB |= (1 << LED_VERMELHO_PIN) | (1 << LED_VERDE_PIN);
    DDRC |= (1 << BUZZER_PIN);
    PORTB &= ~((1 << LED_VERMELHO_PIN) | (1 << LED_VERDE_PIN)); 
    PORTC &= ~(1 << BUZZER_PIN);                             

    DDRB &= ~((1 << MOV_SENSOR_PIN) | (1 << INCLINATION_SENSOR_PIN));
    PORTB |= ((1 << MOV_SENSOR_PIN) | (1 << INCLINATION_SENSOR_PIN)); 

    DDRC &= ~(1 << NIVEL_OLEO_SENSOR_PIN);
    PORTC |= (1 << NIVEL_OLEO_SENSOR_PIN); 

    PCICR |= (1 << PCIE0); 
    PCMSK0 |= (1 << PCINT4) | (1 << PCINT3);
    

    sei();

    if (check_oled_presence(0x3C)) { 
        u8g.setColorIndex(1); 
        show_startup_screen_chao();
        uart_print("OLED Inicializado!\r\n");
    } else {
        uart_print("OLED NAO ENCONTRADO!\r\n");
    }

    uart_print("Master (Chao de Fabrica) pronto.\r\n");

    uint32_t last_i2c_send_time = 0;
    uint32_t last_i2c_request_time = 0;
    uint32_t last_sensor_read_time = 0;
    uint32_t last_debug_print_time = 0; 

    const uint32_t i2c_send_interval = 300; // ms
    const uint32_t i2c_request_interval = 450; 
    const uint32_t sensor_read_interval = 200;  
    const uint32_t debug_print_interval = 2000;  

    verificar_condicoes_producao();
    controlar_leds();
    update_display_chao();


    while (1) {
        uint32_t current_time = get_ms_counter();

        handle_buzzer(); 
        tratar_botao_parada_chao();

        if (current_time - last_sensor_read_time >= sensor_read_interval) {
            last_sensor_read_time = current_time;
            current_temperature = (adc_read_averaged(0, ADC_SAMPLES_TO_AVERAGE) * 500.0) / 1024.0;

            verificar_condicoes_producao();
            cortar_madeira();               
            controlar_leds();             
            update_display_chao();          // atualiza display 
        }

        if (current_time - last_i2c_send_time >= i2c_send_interval) {
            send_status_to_supervisor();
            last_i2c_send_time = current_time;
        }
        if (current_time - last_i2c_request_time >= i2c_request_interval) {
            request_commands_from_supervisor();
            last_i2c_request_time = current_time;
        }

        controlar_motores_dc();

        if (current_time - last_debug_print_time >= debug_print_interval) {
            last_debug_print_time = current_time;
            uart_print("Temp:"); uart_print_float(current_temperature,1);
            uart_print(" Blocos:"); uart_print_num(block_count);
            uart_print(" Velocidade_V_cmd:"); uart_print_num(cmd_velocidade_motor_vertical);
            uart_print(" Velocidade_H_cmd:"); uart_print_num(cmd_velocidade_motor_horizontal);
            uart_print(" Parada_Stop:"); uart_print_num(producao_parada_status);
            uart_print(" Mov:"); uart_print_num(mov_sensor_status);
            uart_print(" Incl:"); uart_print_num(inclination_sensor_status);
            uart_print(" Oleo:"); uart_print_num(nivel_oleo_critico_status);
            uart_print("\r\n");
        }
    }
    return 0; 
}