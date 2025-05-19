#include <Arduino.h>

void setup() {
  // Configura o PB5 (pino 13) como saída
  DDRB |= (1 << DDB5); 
}

void loop() {
  // Liga o LED (coloca PB5 em nível alto)
  PORTB |= (1 << PORTB5);  
  delay(500);

  // Desliga o LED (coloca PB5 em nível baixo)
  PORTB &= ~(1 << PORTB5); 
  delay(500);
}
