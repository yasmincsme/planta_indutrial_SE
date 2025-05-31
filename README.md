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

A automação industrial tem se consolidado como uma ferramenta essencial para o aumento da produtividade, padronização e segurança nos processos produtivos. Neste contexto, o presente projeto propõe o desenvolvimento de um sistema de controle automatizado para um processo industrial de corte de blocos de madeira.

O sistema simula uma linha de produção onde blocos de madeira são transportados por uma esteira e cortados por serras elétricas automatizadas, garantindo precisão e padronização no corte. A arquitetura do sistema é composta por dois microcontroladores Arduino, com funções bem definidas e distribuídas:

* **Arduino Supervisor:** Atua como uma central de gerenciamento, enviando comandos e monitorando o status geral da planta.
* **Arduino Chão de Fábrica:** Controla diretamente os motores de corte, lê dados de sensores (temperatura, inclinação da madeira, presença de operadores, nível de óleo) e opera alertas.

Ambos se comunicam via **I2C**, com o "Chão de Fábrica" como mestre e o "Supervisor" como escravo. Além disso, todo o projeto foi desenvolvido em **linguagem C, manipulando os registradores do ATmega328p**.

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

## Requisitos do Problema

**RQ1 (Código em C Nível Registrador):** O código deve ser desenvolvido em linguagem C diretamente a nível de registrador, sem funções do Arduino;

**RQ1 (Arquitetura com Dois Arduinos):** O sistema possui dois Arduinos: um supervisor (Arduino 1) e outro no chão de fábrica (Arduino 2);

**RQ3 (Comunicação Serial):** Os Arduinos devem se comunicar via um protocolo padrão (UART, SPI ou I2C);

### Arduino 1 (Supervisor)

**4.2.1.1 (Interruptor de “Parada Solicitada”):** A produção é interrompida através do botão conectado ao pino de interrupção D2; 

**4.2.1.2 (Controle dos motores via potenciômetro):** O potenciômetro A1 controla o motor 1 e o potenciômetro A2 controla o motor 2, ambos situados no chão de fábrica. O controle de velocidade é feito através dos pinos ENABLE da ponte H. Além disso, quando os motores estão mais lentos, a velocidade de produção também diminui;

**4.2.1.3. (Comunicação entre o supervisor e o chão de fábrica):** Pode ser verificado através do envio de mensagens, controle dos motores e parada da produção;

**4.2.1.4. (Monitoramente de informações do chão de fábrica no arduino supervisor):** Visualização do status dos sensores e da produção;

- Status do Sensor de Temperatura;
- Status do Sensor de Inclinação;
- Status do Sensor de Presença;
- Status do Nível do Tanque de Óleo;
- Status da Produção;
- Velocidade dos motores;
- Quantidade de blocos de madeiras cortados;

### Arduino 2 (Chão de fábrica)

**4.2.2.1. (Interruptor de parada para interromper a produção):** Caso o interruptor seja acionado através do clique, a produção é reiniciada e a comunicação é interrompida;

**4.2.2.2. (Sensor de temperatura):** Caso a temperatura esteja crítica (acima de 40), o buzzer é acionado e a produção é interrompida, além de exibir no monitor serial a mensagem “Temperatura crítica”. Para a produção voltar ao normal, a temperatura precisa estabilizar e o interruptor do chão de fábrica tem que ser acionado;

**4.2.2.3. (Sensor de inclinação):** Ao detectar inclinação, a produção é interrompida e o led amarelo é ligado, sendo que este último corresponde ao servo motor. Além disso, informações sobre o status do sensor de inclinação são exibidas no monitor serial;

**4.2.2.4. (Motores CC):** Motores conectados ao Chão de Fábrica para corte da madeira;

**4.2.2.5. (Exibição de blocos cortados no display):** Display OLED exibindo informações sobre a produção e status do chão de fábrica;

**4.2.2.6. (Sensor de presença):** A produção só estará ativa e os motores em funcionamento caso o sensor de presença não detecte movimento;

**4.2.2.7. (Controle de LEDS):** Enquanto a produção está ativa, o LED laranja é aceso. Caso a produção seja interrompida, o LED vermelho é ligado.

**4.2.2.8. (Sensor de nível):** O sensor de nível foi substituído por um sensor de distância infravermelho. Assim, a todo momento é possível monitorar o nível do tanque.

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
* Gerencia sensores de temperatura (A0), inclinação (D11), presença (D12) e nível de óleo.
* Controla motores de corte (PWM em D5, D6) e LEDs de status/alerta (D8, D10, D3).
* Aciona um buzzer (A3) para alertas críticos.
* Exibe informações no display OLED (contagem de blocos, status).
* Envia pacotes de status para o Supervisor e solicita comandos.

![-----------------------------------------------------](https://github.com/nailasuely/breakout-problem3/blob/main/assets/img/prancheta.png)

## Sensores

### Sensor de Movimento (Chave DIP 1)

No contexto deste projeto, o sensor de movimento foi representado por uma chave DIP (Dual In-line Package), especificamente a chave DIP 1. Esta chave atua como um sensor digital, simulando a detecção de movimento de forma manual. Quando acionada, ela envia um sinal lógico alto (HIGH) para o microcontrolador, representando a presença de movimento. Quando desacionada, envia um sinal lógico baixo (LOW), indicando ausência de movimento. Essa abordagem facilita a simulação de diferentes cenários durante o desenvolvimento e os testes do sistema.

<div align="center">
   <img width="" src="https://github.com/yasmincsme/planta_indutrial_SE/blob/main/assets/switch_dip.png" />
    <p> Exemplos de chaves DIP</p>
</div>
</p>

### Sensor de Inclinação (Chave DIP 2)

De maneira semelhante ao sensor de movimento, o sensor de inclinação foi representado pela chave DIP 2. Essa chave permite simular situações em que o dispositivo sofre uma inclinação além de um determinado limite. Quando a chave DIP 2 está acionada, o sistema interpreta que há inclinação, então o sistema é desativado e há o acionamento do motor de correção de eixo, representado no circuito por um segundo LED vermelho. 

### Sensor de Temperatura (Potenciômetro)

O potenciômetro foi utilizado para simular a variação de temperatura no sistema. Este componente gera um sinal analógico, cuja tensão varia proporcionalmente à sua posição mecânica. O microcontrolador lê essa variação através de um pino analógico e interpreta os valores como diferentes níveis de temperatura, isso permite testar as respostas do sistema a diferentes condições térmicas.

<div align="center">
   <img width="" src="https://png.pngtree.com/png-vector/20241030/ourmid/pngtree-adjustable-rotary-potentiometer-component-with-metal-knob-for-electrical-circuits-and-png-image_14201974.png" />
    <p> Potenciômetro de 1K</p>
</div>
</p>

### Sensor de Nível (Infrared Sharp GP2D12)

O sensor Sharp GP2D12 é um sensor infravermelho de distância que funciona através de triangulação óptica. Ele mede a distância de objetos dentro de uma faixa de aproximadamente 10 a 80 cm. No projeto, ele foi utilizado para simular um sensor de nível, detectando, por exemplo, o nível de enchimento do reservatório de óleo. O sinal de saída é analógico, e sua leitura permite que o sistema interprete diferentes níveis conforme a distância detectada.

<div align="center">
   <img width="" src="https://d229kd5ey79jzj.cloudfront.net/137/images/137_1_X.png?20250218090419" />
    <p> Sensor sharp GP2Y0A02YK0F</p>
</div>
</p>

![-----------------------------------------------------](https://github.com/nailasuely/breakout-problem3/blob/main/assets/img/prancheta.png)

## Implementação

### Comunicação entre Arduino 1 e Arduino 2

A comunicação entre os dois microcontroladores Arduino foi feita por meio de comunicação serial (UART). Essa interface permite o envio e recebimento de dados de forma eficiente, utilizando os pinos TX (transmissor) e RX (receptor) de cada placa. Através desse canal, as placas trocam informações dos sensores e comandos de controle, garantindo a operação sincronizada do sistema.

### Comunicação com Display OLED

O Display OLED utilizado no projeto se comunica com o Arduino por meio do protocolo I2C, um barramento serial de dois fios (SDA e SCL). Esse protocolo permite a comunicação com múltiplos dispositivos utilizando apenas dois pinos, além da alimentação. O Display OLED é utilizado para apresentar informações importantes, como o valor do sensor de temperatura, estados dos motores, número de blocos produzidos e alertas do sistema, proporcionando uma interface visual simples e eficaz.

### Controle de Motores via PWM

O controle dos motores foi realizado utilizando a técnica de PWM (Pulse Width Modulation). Essa técnica permite ajustar a velocidade dos motores variando o ciclo de trabalho do sinal PWM, ou seja, a proporção de tempo em que o sinal permanece em nível alto dentro de um período.

Para permitir que o microcontrolador controle motores, que normalmente operam em tensões e correntes maiores, foi utilizada uma ponte H. A ponte H é um circuito que permite o controle do sentido de rotação dos motores (horário e anti-horário), além de possibilitar o acionamento ou desligamento. Combinada com o PWM, ela permite controle total sobre velocidade e direção. No projeto, a ponte H garante que os motores possam ser acionados de acordo com as leituras dos sensores e os comandos do sistema.

### Leitura de Pinos Analógicos

Os sensores que fornecem sinais analógicos, como o potenciômetro (sensor de temperatura simulado) e o sensor Sharp GP2D12 (sensor de nível), são conectados aos pinos analógicos do Arduino. Através do conversor analógico-digital (ADC) embutido no microcontrolador, esses sinais são convertidos de tensões analógicas (0V a 5V) para valores digitais que variam de 0 a 1023 (resolução de 10 bits). Esses valores são então processados pelo código para determinar o estado dos sensores e tomar decisões de controle no sistema.

No entanto, a leitura de sinais analógicos apresenta algumas dificuldades. Esses sinais estão sujeitos a ruídos elétricos provenientes tanto do ambiente quanto do próprio circuito, o que pode gerar oscilações e leituras imprecisas. Além disso, a resolução limitada do ADC impacta diretamente na precisão da medição — cada incremento representa cerca de 4,88 mV em uma conversão de 10 bits, o que pode ser insuficiente para aplicações que exigem maior sensibilidade.

Outros fatores, como instabilidade na alimentação, interferências eletromagnéticas, variações de temperatura e não linearidades dos sensores, também podem comprometer a confiabilidade da leitura. Diante dessas dificuldades, optamos por utilizar potenciômetros e chaves (botões ou sensores digitais) para a leitura dos sensores no projeto. Essa escolha visa simplificar o sistema, reduzir os impactos do ruído e aumentar a robustez e a confiabilidade das medições, uma vez que os sinais digitais são menos suscetíveis a variações e interferências.

## Temporização

A temporização no projeto foi implementada utilizando os timers internos dos microcontroladores dos Arduinos, configurados diretamente a nível de registrador. Essa abordagem garante maior precisão e controle sobre os intervalos de tempo, evitando o uso de funções como *delay()* que bloqueiam o funcionamento do programa. Os timers foram configurados no modo CTC (Clear Timer on Compare), permitindo que a cada estouro, uma interrupção seja gerada. Dentro dessas rotinas de interrupção, são atualizadas variáveis de contagem de tempo, que funcionam como bases para temporizar diferentes eventos do sistema.

Esse controle de tempo foi essencial para diversas funções dentro da automação da planta industrial simulada. A lógica de detecção do clique duplo no botão de parada local, por exemplo, só foi possível graças à temporização, que define uma janela de tempo específica para diferenciar um clique simples de um duplo clique. Além disso, a temporização foi aplicada no controle da esteira e das serras, garantindo que o transporte do bloco e o acionamento das lâminas ocorram de forma sincronizada e segura, respeitando os tempos necessários para posicionamento correto da madeira antes do corte.

Outra aplicação importante da temporização está no gerenciamento dos alarmes de segurança. No caso de temperatura crítica, o buzzer permanece ativo enquanto a condição de risco persiste, e isso é controlado por contadores de tempo que garantem que o alerta não seja apenas um pulso instantâneo, mas sim um sinal sonoro constante até que o problema seja resolvido. A atualização das informações no display OLED também depende da temporização, que regula a frequência de escrita para evitar travamentos na comunicação I2C e garantir uma exibição estável dos dados.

![-----------------------------------------------------](https://github.com/nailasuely/breakout-problem3/blob/main/assets/img/prancheta.png)

## Testes

**Funcionamento Geral do Sistema:**
O sistema inicia corretamente, com todas as informações sendo exibidas no monitor serial. Os motores funcionam normalmente, e o LED laranja é aceso, indicando que a produção está ativa.

<div align="center">
   <img width="" src="https://github.com/yasmincsme/planta_indutrial_SE/blob/main/assets/teste_geral.gif" />
    <p> Funcionamento Geral do Sistema</p>
</div>
</p>

**Controle pelo Botão do Arduino Supervisor:**
O botão do supervisor controla o funcionamento do Arduino no chão de fábrica. Quando pressionado, a produção é interrompida: os motores desligam, o LED vermelho acende e o display OLED exibe a mensagem indicando que a produção foi pausada. Ao pressionar novamente, a produção é retomada — os motores voltam a funcionar e o LED laranja acende, sinalizando o retorno à operação normal.

<div align="center">
   <img width="" src="https://github.com/yasmincsme/planta_indutrial_SE/blob/main/assets/botao_supervisor.gif" />
    <p> Botão do Arduino Supervisor</p>
</div>
</p>

**Controle pelo Botão do Chão de Fábrica:**
O botão localizado no chão de fábrica funciona de maneira semelhante ao botão do supervisor, permitindo pausar e retomar a produção. A diferença é que, mesmo que a produção tenha sido interrompida pelo chão de fábrica, o supervisor tem autoridade para religar o sistema.

<div align="center">
   <img width="" src="https://github.com/yasmincsme/planta_indutrial_SE/blob/main/assets/botao_chao.gif" />
    <p> Botão do Chão de Fábrica</p>
</div>
</p>

**Controle do Motor 1:**
O potenciômetro 1 permite ajustar a velocidade de giro do motor 1. Esse ajuste influencia diretamente a velocidade de produção, permitindo aumentar ou diminuir conforme a necessidade.

<div align="center">
   <img width="" src="https://github.com/yasmincsme/planta_indutrial_SE/blob/main/assets/motor_1.gif" />
    <p> Controle do Motor 1</p>
</div>
</p>

**Controle do Motor 2:**
Da mesma forma, o potenciômetro 2 controla a velocidade de giro do motor 2, também impactando diretamente na velocidade da linha de produção.

<div align="center">
   <img width="" src="https://github.com/yasmincsme/planta_indutrial_SE/blob/main/assets/motor_2.gif" />
    <p> Controle do Motor 2</p>
</div>
</p>

**Parada por Temperatura Crítica:**
Se a temperatura atinge valores fora da faixa de operação, o sistema entra em estado de segurança: os motores são desligados, o LED vermelho acende, e uma mensagem de alerta é exibida tanto no display OLED quanto no monitor serial. A produção só é retomada quando a temperatura volta ao valor adequado.

<div align="center">
   <img width="" src="https://github.com/yasmincsme/planta_indutrial_SE/blob/main/assets/temperatura_critica.gif" />
    <p> Temperatura Crítica</p>
</div>
</p>

**Sensor de Inclinação:**
Quando o sensor detecta uma inclinação anormal, o sistema interrompe imediatamente a produção. Além disso, o motor de correção de eixo é acionado, representado no protótipo pelo acendimento de um LED vermelho adicional.

<div align="center">
   <img width="" src="https://github.com/yasmincsme/planta_indutrial_SE/blob/main/assets/inclinacao.gif" />
    <p> Sensor de Inclinação</p>
</div>
</p>

**Sensor de Movimento:**
Ao detectar movimento inesperado, o sensor interrompe automaticamente a produção como medida de segurança. Quando o movimento indesejado cessa, o sistema retoma a produção normalmente. O funcionamento é análogo ao do sensor de inclinação.

<div align="center">
   <img width="" src="https://github.com/yasmincsme/planta_indutrial_SE/blob/main/assets/movimento.gif" />
    <p> Sensor de Movimento</p>
</div>
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
