#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(9,10);     // CE = 9, CSN = 10  (pode alterar se quiser)
const byte endereco[6] = "canal1";
struct PacoteJoystick {
  uint16_t rx;
  uint16_t ry;
};
PacoteJoystick pacote;
void setup() {
  analogReference(DEFAULT);
  radio.begin();
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(endereco);
  radio.stopListening(); // modo TX
}
void loop() {
  pacote.rx = analogRead(A1);  // curva
  pacote.ry = analogRead(A0);  // velocidade
  radio.write(&pacote, sizeof(pacote));  
}