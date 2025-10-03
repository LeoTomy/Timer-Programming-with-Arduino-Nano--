#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// Globale Variable für LED-Zustand und Zähler (für D13)
volatile uint8_t led_state = 0;
volatile uint16_t timer_count = 0;
volatile uint16_t adc_value_global = 0; // Global ADC value for ISR

// Globale Variable für Dimmen der LED an D5
volatile uint8_t brightness = 0;
volatile int8_t direction = 1; // 1 für heller, -1 für dunkler

// Funktion zur Initialisierung des ADC
void adc_init(void) {
    // AVCC als Referenzspannung, ADC0 (PC0) ausgewählt
    ADMUX = (1 << REFS0);
    // ADC aktivieren, Prescaler 128 (16 MHz / 128 = 125 kHz)
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

// Funktion zum Lesen des ADC-Wertes
uint16_t adc_read(uint8_t channel) {
    // Kanal auswählen (MUX-Bits setzen)
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
    // Conversion starten
    ADCSRA |= (1 << ADSC);
    // Warten bis Conversion abgeschlossen
    while (ADCSRA & (1 << ADSC));
    // ADC-Wert zurückgeben
    return ADC;
}

// Funktion zur Initialisierung des Servos (Timer1, Pin D9 - PB1)
void servo_init(void) {
    // PB1 (OC1A) als Ausgang
    DDRB |= (1 << PB1);
    
    // Timer1: Fast PWM, 8-bit, Top = ICR1
    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11); // Prescaler 8
    
    // Top-Wert für 20ms Periode (50 Hz) bei 16 MHz und Prescaler 8
    ICR1 = 40000; // (16 MHz / 8) * 0.02s = 40000
    
    // Startposition (0.5ms = 0°)
    OCR1A = 1000;
}

// Funktion zum Setzen des Servo-Winkels (0° bis 180°)
void servo_set_angle(uint8_t angle) {
    // Berechnung des PWM-Wertes: 0.5ms (0°) bis 2.5ms (180°)
    // 0.5ms = 1000, 2.5ms = 5000
    uint16_t pwm = 1000 + ((uint32_t)angle * 4000) / 180;
    OCR1A = pwm;
}

// Funktion zur Initialisierung von Timer0 für PWM an D5
void timer0_init(void) {
    // PB5 (D13) als Ausgang (onboard LED)
    DDRB |= (1 << PB5);
    // PD5 (D5) als Ausgang für PWM (OC0B)
    DDRD |= (1 << PD5);
    
    // Timer0: Fast PWM, Non-inverting mode für OC0B
    TCCR0A = (1 << COM0B1) | (1 << WGM01) | (1 << WGM00); // Fast PWM, 8-bit
    TCCR0B = (1 << CS02) | (1 << CS00); // Prescaler 1024
    
    // Overflow Interrupt aktivieren
    TIMSK0 |= (1 << TOIE0);
    
    // Startwerte
    OCR0B = 0; // Anfangshelligkeit
    PORTB &= ~(1 << PB5); // D13 LED aus
}

ISR(TIMER0_OVF_vect) {
    // Dimmen der LED an D5
    brightness += direction; // Schrittweite 1 für glatte Übergänge
    if (brightness == 255) {
        direction = -1; // Beginne mit dem Dunklerwerden
    } else if (brightness == 0) {
        direction = 1; // Beginne mit dem Hellerwerden
    }
    OCR0B = brightness; // PWM-Wert für D5 setzen
    
    // Blinken der LED an D13 nur bei ADC > 511
    if (adc_value_global > 511) {
        timer_count++;
        // 500 ms = 0,5 s / 0,016384 s ≈ 30,52 Zyklen
        if (timer_count >= 31) {
            led_state = !led_state;
            if (led_state) {
                PORTB |= (1 << PB5); // D13 LED an
            } else {
                PORTB &= ~(1 << PB5); // D13 LED aus
            }
            timer_count = 0; // Zähler zurücksetzen
        }
    } else {
        PORTB &= ~(1 << PB5); // D13 LED aus, wenn ADC <= 511
        timer_count = 0; // Zähler zurücksetzen, um bei erneuter Aktivierung von 0 zu starten
    }
}

int main(void) {
    // Initialisierungen
    adc_init();
    servo_init();
    timer0_init();
    
    // Interrupts global aktivieren
    sei();
    
    uint16_t adc_value;
    uint8_t angle;
    
    while (1) {
        // ADC-Wert lesen (0-1023)
        adc_value = adc_read(0);
        
        // ADC-Wert für ISR aktualisieren
        adc_value_global = adc_value;
        
        // Umrechnung in Winkel (0-180°)
        angle = ((uint32_t)adc_value * 180) / 1023;
        
        // Servo-Winkel setzen
        servo_set_angle(angle);
        
        // Kurze Pause für Stabilität
        _delay_ms(20);
    }
    
    return 0;
}
