<h1 align="center">
  <br>
    <img width="200x" src="https://github.com/nailasuely/planta_industrial_SE/blob/main/assets/logo.png"> 
  <br>
  Controle de uma planta industrial
  <br>
</h1>


<h4 align="center">Projeto da disciplina TEC 470 - Sistemas Embarcados </h4>

<p align="center">
<div align="center">

[![MIT License](https://img.shields.io/badge/license-MIT-blue.svg)](https://github.com/nailasuely/planta_industrial_SE/blob/main/LICENSE)


> Esse é um projeto da disciplina TEC 470 - Sistemas Embarcados, no qual ocorre o desenvolvimento de  um sistema de controle industrial para corte automatizado de blocos de madeira, utilizando dois Arduinos Nano que se comunicam via protocolo I2C. O sistema monitora sensores e controla motores.
> > 
## Download do repositório

```
gh repo clone nailasuely/planta_indutrial_SE
```

</div>

<details open="open">
<summary>Sumário</summary>

- [Sobre o Projeto](#sobre-o-projeto)
- [Tecnologias e Ferramentas Utilizadas](#Tecnologias-e-Ferramentas-Utilizadas)
- [Arquitetura do Sistema](#Arquitetura-do-Sistema)
- [Testes](#Testes)
- [Equipe](#equipe)
- [Tutor](#tutor)
- [Referências](#referências)
  
</details>

![-----------------------------------------------------](https://github.com/nailasuely/breakout-problem3/blob/main/assets/img/prancheta.png)

## Sobre o Projeto

Este projeto simula um sistema de controle para uma linha de corte de madeira automatizada. Utilizamos dois Arduinos Nano:

* **Arduino Supervisor:** Atua como uma central de gerenciamento, enviando comandos e monitorando o status geral da planta.
* **Arduino Chão de Fábrica:** Controla diretamente os motores de corte, lê dados de sensores (temperatura, inclinação da madeira, presença de operadores, nível de óleo) e opera alertas.

Ambos se comunicam via **I2C**, com o "Chão de Fábrica" como mestre e o "Supervisor" como escravo. O foco foi desenvolver em **linguagem C pura, manipulando registradores do ATmega328p**.
![-----------------------------------------------------](https://github.com/nailasuely/breakout-problem3/blob/main/assets/img/prancheta.png)


##  Tecnologias e Ferramentas

* **Microcontroladores:** 2x Arduino Nano (ATmega328p)
* **Linguagem & Ambiente:**
    * C (com foco em registradores AVR)
    * Arduino IDE / VSCode com PlatformIO
    * Bibliotecas: `Wire.h` (I2C), `U8glib.h` (OLED)
* **Comunicação:** Protocolo I2C
* **Sensores (Simulados/Reais):**
    * Potenciômetros (Controle de velocidade)
    * Interruptores (Parada de emergência)
    * Sensor de Temperatura (LM35 via ADC e Potenciômetro)
    * Sensor de Inclinação (Chave dip switch)
    * Sensor de Presença (PIR/Switch digital)
    * Sensor de Nível de Óleo (Chave dip switch)
* **Atuadores:**
    * Motores CC
    * LEDs (Status, Alertas)
    * Buzzer (Alertas sonoros)
    * Display OLED SSD1306
* **Hardware Adicional:** Protoboards, jumpers, resistores.
![-----------------------------------------------------](https://github.com/nailasuely/breakout-problem3/blob/main/assets/img/prancheta.png)

## Arquitetura do Sistema

O sistema opera com dois Arduinos Nano em uma configuração Master/Slave via I2C:

### Arduino 1: Supervisor (Slave I2C)
* **Endereço I2C:** 9
* Recebe um botão de parada de emergência (D2).
* Lê dois potenciômetros (A0, A1) para definir as velocidades dos motores.
* Envia comandos de velocidade e status de parada ao Master quando solicitado.
* Recebe e exibe dados detalhados do Chão de Fábrica no Monitor Serial a cada 3 segundos.

### Arduino 2: Chão de Fábrica (Master I2C)
* Inicia toda a comunicação I2C.
* Possui um botão de parada local (D2) com função de reinício por duplo clique.
* Gerencia sensores de temperatura (A0), inclinação (D11), presença (D12) e nível de óleo (A4 - **Requer atenção, veja "Como Utilizar"**).
* Controla motores de corte (PWM em D5, D6) e LEDs de status/alerta (D8, D10, D3).
* Aciona um buzzer (A3) para alertas críticos.
* Exibe informações no display OLED (contagem de blocos, status).
* Envia pacotes de status para o Supervisor e solicita comandos.

![-----------------------------------------------------](https://github.com/nailasuely/breakout-problem3/blob/main/assets/img/prancheta.png)

## Testes

<div align="center">
   <img width="" src="https://github.com/nailasuely/" />
    <p> Fig X. Teste</p>
</div>
texto aqui 
</p>


![-----------------------------------------------------](https://github.com/nailasuely/breakout-problem3/blob/main/assets/img/prancheta.png)

## Equipe

- Naila Suele
- Yasmin Cordeiro

## Tutor

- Wild Freitas

</div>

![-----------------------------------------------------](https://github.com/nailasuely/breakout-problem3/blob/main/assets/img/prancheta.png)

## Referências  
> - [1] Microchip Technology Inc. "ATmega328P Datasheet."  
> - [2] Arduino. "Wire.h Library Documentation." https://www.arduino.cc/en/Reference/Wire  
> - [3] U8glib. "U8glib Library Documentation." https://github.com/olikraus/u8glib  
