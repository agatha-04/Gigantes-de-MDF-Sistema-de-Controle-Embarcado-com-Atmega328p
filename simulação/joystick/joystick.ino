#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
 
#define THRESHOLD 600
#define MAX_PWM 255
 
volatile uint8_t leds_state = 0b00001011; // PD0,PD1,PD3
volatile uint8_t pausa = 0;
volatile uint8_t pausa_contador = 0;
 
// Timer1: laser piscando 1Hz
ISR(TIMER1_COMPA_vect)
{
    if (pausa)
    {
        pausa_contador++;
        PORTC &= ~(1<<PC4);
        return;
    }
    PORTC ^= (1<<PC4);
}
 
// Inicializa Timer1
void timer1_init()
{
    TCCR1B |= (1<<WGM12);
    OCR1A = 15624;
    TCCR1B |= (1<<CS12)|(1<<CS10);
    TIMSK1 |= (1<<OCIE1A);
}
 
// ADC
void adc_init()
{
    ADMUX = (1<<REFS0);
    ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
}
 
uint16_t adc_read(uint8_t ch)
{
    ADMUX = (1<<REFS0) | (ch & 0x07);
    ADCSRA |= (1<<ADSC);
    while(ADCSRA & (1<<ADSC));
    return ADC;
}
 
// LEDs
void apagar_proximo_led()
{
    if (leds_state & (1<<0)) { PORTD &= ~(1<<PD0); leds_state &= ~(1<<0); return; }
    if (leds_state & (1<<1)) { PORTD &= ~(1<<PD1); leds_state &= ~(1<<1); return; }
    if (leds_state & (1<<3)) { PORTD &= ~(1<<PD3); leds_state &= ~(1<<3); return; }
}
 
void resetar_sistema()
{
    PORTD |= (1<<PD0)|(1<<PD1)|(1<<PD3);
    leds_state = 0b00001011;
    pausa = 0;
    pausa_contador = 0;
    PORTC &= ~(1<<PC4);
}
 
// PWM Timer0
void pwm_init()
{
    TCCR0A = (1<<WGM00)|(1<<WGM01)|(1<<COM0A1)|(1<<COM0B1);
    TCCR0B = (1<<CS00);
}
 
// Desliga motores
void motores_parar()
{
    OCR0B = 0; OCR0A = 0;
    PORTB &= ~(1<<PB0); PORTD &= ~(1<<PD7);
    PORTB &= ~(1<<PB4); PORTD &= ~(1<<PD4);
}
 
// Controle motores simples com curvas
void motores_controlar(int16_t rvx, int16_t rvy)
{
    // PWM base proporcional à velocidade
    uint8_t pwm_base = (rvy>512) ? (rvy-512) * MAX_PWM / 512 : (512-rvy) * MAX_PWM / 512;
    if (pwm_base > MAX_PWM) pwm_base = MAX_PWM;
 
    // Ajuste de curva: RVX diminui um motor e aumenta outro
    int16_t diff = rvx - 512; // positivo → curva direita
    int16_t m1 = pwm_base - diff * pwm_base / 512;
    int16_t m2 = pwm_base + diff * pwm_base / 512;
 
    // Limita PWM entre 0 e 255
    uint8_t pwm_m1 = (m1<0)?0:(m1>255?255:m1);
    uint8_t pwm_m2 = (m2<0)?0:(m2>255?255:m2);
 
    // Direção dos motores
    if (rvy >= 512)
    {
        PORTB |= (1<<PB0); PORTD &= ~(1<<PD7); // M1 frente
        PORTB |= (1<<PB4); PORTD &= ~(1<<PD4); // M2 frente
    }
    else
    {
        PORTB &= ~(1<<PB0); PORTD |= (1<<PD7); // M1 ré
        PORTB &= ~(1<<PB4); PORTD |= (1<<PD4); // M2 ré
    }
 
    OCR0B = pwm_m1; // PD5 M1
    OCR0A = pwm_m2; // PD6 M2
}
 
int main(void)
{
    // LEDs
    DDRD |= (1<<PD0)|(1<<PD1)|(1<<PD3);
    PORTD |= (1<<PD0)|(1<<PD1)|(1<<PD3);
 
    // Laser
    DDRC |= (1<<PC4);
    PORTC &= ~(1<<PC4);
 
    // ADC
    adc_init();
 
    // Motores
    DDRD |= (1<<PD5)|(1<<PD6)|(1<<PD4)|(1<<PD7);
    DDRB |= (1<<PB0)|(1<<PB4);
    pwm_init();
 
    // Timer laser
    timer1_init();
    sei();
 
    uint8_t luz_detectada = 0;
 
    while(1)
    {
        // Pausa quando LEDs apagados
        if (!pausa && leds_state==0)
        {
            pausa = 1;
            pausa_contador=0;
            PORTC &= ~(1<<PC4);
            motores_parar();
        }
 
        // LDR
        uint16_t luz = adc_read(3);
        if (!pausa)
        {
            if (luz > THRESHOLD)
            {
                if (!luz_detectada)
                {
                    apagar_proximo_led();
                    luz_detectada = 1;
                }
            }
            else luz_detectada = 0;
        }
 
        // Joystick
        int16_t rvx = adc_read(1);
        int16_t rvy = adc_read(0);
 
        if (!pausa) motores_controlar(rvx,rvy);
        else motores_parar();
 
        // Pausa completa
        if (pausa && pausa_contador >= 5) resetar_sistema();
    }
}