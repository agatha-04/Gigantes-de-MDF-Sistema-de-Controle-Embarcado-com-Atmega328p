#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
 
RF24 radio(9,10);
const byte endereco[6] = "canal1";
 
struct PacoteJoystick {
  uint16_t rx;
  uint16_t ry;
};
 
PacoteJoystick pacote;
 
// --------- DEFINIÇÕES DE PINOS ---------
 
// LEDs
#define LED1 PD0
#define LED2 PD1
#define LED3 PD3
 
// Laser
#define LASER PC4
 
// LDR
#define LDR_CH 3
 
// Motores
#define ENA1_D2 PD5
#define ENA3_D4 PD6
#define IN1 PB0
#define IN2 PD7
#define IN3 PB4
#define IN4 PD4
 
// Botão (USEI PB3 para evitar conflito com IN3)
#define BTN PB3
 
#define THRESHOLD 500
 
// Variáveis
volatile uint8_t pausa = 0;
volatile uint16_t pausa_timer = 0;
 
volatile uint8_t leds_state = 0b00001011;
 
// Timer1: laser 1 Hz
ISR(TIMER1_COMPA_vect) {
 
  if (pausa) {
    pausa_timer++;
    PORTC &= ~(1<<LASER);
    return;
  }
 
  // pisca a 1 Hz
  PORTC ^= (1<<LASER);
}
 
void timer_laser_init() {
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1B |= (1<<WGM12);  // CTC
  OCR1A = 15624;         // 1Hz em 16MHz / 1024
  TCCR1B |= (1<<CS12)|(1<<CS10); // prescaler 1024
  TIMSK1 |= (1<<OCIE1A);
}
 
void pwm_init() {
  TCCR0A = (1<<WGM00)|(1<<WGM01)|(1<<COM0A1)|(1<<COM0B1);
  TCCR0B = (1<<CS00);
}
 
void apagar_proximo_led() {
 
  pausa = 1;
  pausa_timer = 0;
 
  if (leds_state & (1<<0)) { PORTD &= ~(1<<LED1); leds_state &= ~(1<<0); return; }
  if (leds_state & (1<<1)) { PORTD &= ~(1<<LED2); leds_state &= ~(1<<1); return; }
  if (leds_state & (1<<3)) { PORTD &= ~(1<<LED3); leds_state &= ~(1<<3); return; }
}
 
void resetar_sistema() {
  PORTD |= (1<<LED1)|(1<<LED2)|(1<<LED3);
  leds_state = 0b00001011;
  pausa = 0;
  pausa_timer = 0;
  PORTC &= ~(1<<LASER);
}
 
void motores_parar() {
  OCR0B = 0;
  OCR0A = 0;
  PORTB &= ~(1<<IN1); PORTD &= ~(1<<IN2);
  PORTB &= ~(1<<IN3); PORTD &= ~(1<<IN4);
}
 
void motores_controlar() {
 
  uint16_t rvx = pacote.rx;
  uint16_t rvy = pacote.ry;
 
  uint8_t pwm_base = abs((int)rvy - 512) * 255 / 512;
 
  int16_t diff = (int)rvx - 512;
  int16_t m1 = pwm_base - diff * pwm_base / 512;
  int16_t m2 = pwm_base + diff * pwm_base / 512;
 
  uint8_t pwm_m1 = constrain(m1, 0, 255);
  uint8_t pwm_m2 = constrain(m2, 0, 255);
 
  if (rvy >= 512) {
    PORTB |= (1<<IN1); PORTD &= ~(1<<IN2);
    PORTB |= (1<<IN3); PORTD &= ~(1<<IN4);
  } else {
    PORTB &= ~(1<<IN1); PORTD |= (1<<IN2);
    PORTB &= ~(1<<IN3); PORTD |= (1<<IN4);
  }
 
  OCR0B = pwm_m1;
  OCR0A = pwm_m2;
}
 
void setup() {
  DDRD |= (1<<LED1)|(1<<LED2)|(1<<LED3);
  PORTD |= (1<<LED1)|(1<<LED2)|(1<<LED3);
 
  DDRC |= (1<<LASER);
  PORTC &= ~(1<<LASER);
 
  DDRD |= (1<<ENA1_D2)|(1<<ENA3_D4)|(1<<IN4)|(1<<IN2);
  DDRB |= (1<<IN1)|(1<<IN3);
 
  // botão
  DDRB &= ~(1<<BTN);
  PORTB |= (1<<BTN);  // pull-up
 
  pwm_init();
  timer_laser_init();
  sei();
 
  radio.begin();
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_250KBPS);
  radio.openReadingPipe(1, endereco);
  radio.startListening();
}
 
void loop() {
 
  // pausa de 5s depois de APAGAR QUALQUER LED
  if (pausa && leds_state != 0 && pausa_timer >= 5) {
    pausa = 0;
    pausa_timer = 0;
  }
 
  // pausa final: 3 leds apagados → só reinicia no botão
  if (leds_state == 0) {
    motores_parar();
    PORTC &= ~(1<<LASER);
 
    if (!(PINB & (1<<BTN))) { // botão pressionado
      delay(200);
      resetar_sistema();
    }
    return;
  }
 
  // se está em pausa, nada funciona
  if (pausa) {
    motores_parar();
    return;
  }
 
  // rádio
  if (radio.available()) {
    radio.read(&pacote, sizeof(pacote));
  }
 
  // LDR
  uint16_t luz = analogRead(LDR_CH);
  static uint8_t luz_detectada = 0;
 
  if (luz > THRESHOLD) {
    if (!luz_detectada) {
      apagar_proximo_led();
      luz_detectada = 1;
    }
  } else {
    luz_detectada = 0;
  }
 
  motores_controlar();
}