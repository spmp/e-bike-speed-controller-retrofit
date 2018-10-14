#include <Arduino.h>
#include <EEPROM.h>

/** Pin defines **/
#define IN_THROTTLE_ADC_PIN     A7
#define IN_BRAKE_PIN            2
// Hardware allows only pin 9 or 10
#define OUT_PWM_PIN             9
// PWM_MAX define the frequency, set at 9 bits to give ~16KHz
#define PWM_MAX                 0x3FF

/** Serial 0 - debug serial **/
// Serial (0) defines
#define SERIAL0_BAUD            38400

/** EEPROM addresses **/
#define EEPROM_ADDRESS_ADC_CAL  0


/** Throttle calibration constants **/
#define ADC_MAX                 0x3FF
#define ADC_CAL_THRESHOLD       800
#define THROTTLE_LOW_THRESHOLD  100

/** Global variables **/
uint16_t throttle_position_adc  = 0;
float    throttle_position_cal  = 1.0;
uint16_t pwm_input_value        = 0;


/** Functions **/

void setup() {
  Serial.begin(SERIAL0_BAUD);

  /** Set brake input pin as pull up input **/
  pinMode(IN_BRAKE_PIN, INPUT_PULLUP);

  /** Calibrate the throttle **/
  throttle_position_adc         = analogRead(IN_THROTTLE_ADC_PIN);
  // Calibrate if ADC measure is above threshold,
  // i.e throttle is held on when starting
  if (throttle_position_adc >= ADC_CAL_THRESHOLD) {
    Serial.println("Calibrating throttle");
    // Save value to EEROM
    EEPROM.put(EEPROM_ADDRESS_ADC_CAL, float(ADC_MAX) / throttle_position_adc);
  }

  // Read the calibration value out of EEPROM
  EEPROM.get(EEPROM_ADDRESS_ADC_CAL, throttle_position_cal);

  /** Setup PWM output **/
  pinMode(OUT_PWM_PIN, OUTPUT);
  // Set the pin low
  digitalWrite(OUT_PWM_PIN, LOW);

  // Set Timer 1 PWM to phase corrected (X bit) to give 16KHz
  // This means no prescalar and a 9 bit PWM. Starting with 10 bit
  // and using ICR1 as top so we can set it from define above
  // PWM_frequency = clock_speed / (2 * Prescaller_value * TOP_value )

  // Reset Timer 1 registers
  TCCR1A                        = 0;
  TCCR1B                        = 0;
  TCNT1                         = 0;

  // Set TOP to (PWM_MAX)
  ICR1                          = PWM_MAX;
  // Set duty cycle to 0
  OCR1A                         = 0;

  // Set PWM to normal non-inverting mode
  TCCR1A                       |= (1 << COM1A1)|(1 << COM1B1);
  // Set PWM to phase corrected mode with ICR1 as TOP
  TCCR1A                       |= (1 << WGM11);
  TCCR1B                       |= (1 << WGM13);

  // Start the timer and hence the PWM
  TCCR1B                       |= (1 << CS10);

  /** Block exiting setup until throttle has gone back to below threshold **/
  while (analogRead(IN_THROTTLE_ADC_PIN) >= THROTTLE_LOW_THRESHOLD){
    delay(10);
  }

}

void loop() {
    // Read throttle
    throttle_position_adc       = analogRead(IN_THROTTLE_ADC_PIN);

    // Find PWM input and clamp at 1024, or 0 if brake is on
    pwm_input_value             = (throttle_position_adc * throttle_position_cal / ADC_MAX) * PWM_MAX;
    if (digitalRead(IN_BRAKE_PIN) == LOW ) {
      pwm_input_value           = 0;
    }
    if (pwm_input_value > PWM_MAX) {
      pwm_input_value           = PWM_MAX;
    }

    // Set the PWM output
    OCR1A = pwm_input_value;

    Serial.print("Throttle position: ");
    Serial.print(throttle_position_adc);
    Serial.print(" Calibration constant: ");
    Serial.print(throttle_position_cal);
    Serial.print(". Calibrated output: ");
    Serial.println(pwm_input_value);

    delay(100);
}
