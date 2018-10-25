#include <Arduino.h>

/** Pin defines **/
#define IN_THROTTLE_ADC_PIN     A7
#define IN_BRAKE_PIN            2
#define IN_BRAKE_ON             LOW
// Hardware allows only pin 9 or 10
#define OUT_PWM_PIN             9
// PWM_MAX define the frequency, set at 9 bits to give ~16KHz
#define PWM_MAX                 0xFFF

/** Serial 0 - debug serial **/
// Serial (0) defines
#define SERIAL0_BAUD            38400

/** Throttle calibration constants **/
#define ADC_MAX                 0x3FF
#define ADC_THROTTLE_LOW        205
#define ADC_THROTTLE_HIGH       690

/** Ramp (up) constant **/
#define RAMP_PROPORTION         0.001
#define RAMP_MAX_PWM_INCREASE   PWM_MAX*RAMP_PROPORTION

/** Functions **/

/** calculate_pwm_input
 * transform the throttle reading to a value between 0 and PWM_MAX for the
 * range ADC_THROTTLE_LOW to ADC_THROTTLE_HIGH
 **/
uint16_t calculate_pwm_input(
  uint16_t adc_read,
  bool brake_input,
  uint16_t adc_throttle_low,
  uint16_t adc_throttle_high,
  uint16_t pwm_max,
  uint16_t pwm_delta_max
){
  // Initialise static function variable used for ramp
  static uint16_t pwm_input_prev = 0;
  // Initialise static function valriable used to hold temporary return value
  static uint16_t pwm_input = 0;

  // Clamp input between 0 and pwm_max in the input range adc_throttle_low to
  // adc_throttle_high.
  // Zero the input value if the brake is on
  // If adc_read is between low and high map it onto the range 0 to pwm_max
  if (adc_read <= adc_throttle_low || brake_input == true) {
    pwm_input = 0;
  } else if (adc_read >= adc_throttle_high){
    pwm_input = pwm_max;
  } else {
    pwm_input =  uint16_t(
      ((float(adc_read) - adc_throttle_low)/
        (adc_throttle_high - adc_throttle_low))*
        pwm_max
      );
  }

  // Apply a ramp to the pwm_input, ensuring that the increase in pwm_input
  // is never greater than the previous value + pwm_delta_max
  if (pwm_input > pwm_input_prev + pwm_delta_max) {
    pwm_input = pwm_input_prev + pwm_delta_max;
  }
  // Update the previouse pwm_input
  pwm_input_prev = pwm_input;

  // Return the pwm_input
  return pwm_input;

}

bool check_brake(uint8_t brake_pin, uint8_t on_value){
  if (digitalRead(brake_pin) == on_value) {
    return true;
  } else {
    return false;
  }
}

void setup() {
  // Set PWM pin output and low at turn on
  pinMode(OUT_PWM_PIN, OUTPUT);
  digitalWrite(OUT_PWM_PIN, LOW);

  Serial.begin(SERIAL0_BAUD);

  /** Set brake input pin as pull up input **/
  pinMode(IN_BRAKE_PIN, INPUT_PULLUP);

  /** Setup PWM output **/
  // refer to https://sites.google.com/site/qeewiki/books/avr-guide/pwm-on-the-atmega328
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
  TCCR1B                        |= (1 << CS10);

}

void loop() {
  // The variables
  static unsigned long mileycyrus        = 0;
  static uint16_t throttle_position_adc  = 0;
  static bool     brake_on               = true;
  static uint16_t pwm_input_value        = 0;

  // Run various tasks at different cadences.
  // We update the PWM duty cycle every 100th of a second (every 10 millis)
  // and we update the display every quarter of a second
  mileycyrus = millis();
  // Every 10 milliseconds
  if (mileycyrus % 10 == 0) {
    // Find PWM input to the PWM thingimiginator
    throttle_position_adc       = analogRead(IN_THROTTLE_ADC_PIN);
    brake_on                    = check_brake(IN_BRAKE_PIN, IN_BRAKE_ON);
    pwm_input_value             = calculate_pwm_input(
                                    throttle_position_adc,
                                    brake_on,
                                    ADC_THROTTLE_LOW,
                                    ADC_THROTTLE_HIGH,
                                    PWM_MAX,
                                    RAMP_MAX_PWM_INCREASE
                                  );

    // Set the PWM output
    OCR1A                       = pwm_input_value;
  }
  // every 100 milliseconds
  if (mileycyrus % 100 == 0) {
    Serial.print("Millisomethings: ");
    Serial.print(mileycyrus);
    Serial.print(" Throttle position: ");
    Serial.print(throttle_position_adc);
    Serial.print(" Brake on: ");
    Serial.print(brake_on);
    Serial.print(". Calibrated output: ");
    Serial.println(pwm_input_value);
  }
}
