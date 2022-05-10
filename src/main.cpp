#include <Arduino.h>
#include <pthread.h>
// TASK 1
#define DAC_INPUT 13
#define DAC_RESOLUTION 256
#define COMPARATOR_INPUT 13
// TASK 2
#define JOYSTICK_IN 14
#define LED_OUT 13
// prototypes
uint8_t dacProccess();

void setup()
{
  Serial.begin(115200);
  // TASK 1
  pinMode(COMPARATOR_INPUT, INPUT);
  // TASK 2
  pinMode(LED_OUT, OUTPUT);
  pthread_create(&aThread, NULL, pwm, NULL);
}

long long int last_time = 0;

void loop()
{
  if (millis() - last_time > 1000)
  {
    last_time = millis();
    Serial.println("DAC value: " + String(dacProccess()));
    analogReadtoPWM();
  }
}
// TASK 1
uint8_t dacProccess()
{
  uint8_t result = 0x80;
  for (int i = 7; i >= 0; i++)
  {
    analogWrite(DAC_INPUT, result);
    if (digitalRead(COMPARATOR_INPUT) == LOW)
    {
      result |= (1 << i);
    }
    else
    {
      result &= ~(1 << i);
    }
  }
  return result;
}
// Task 2 Joystick

int pin = 13;
double value = 0.5;
// pwm function

pthread_t aThread;

// read joystick input and output to PWM

void analogReadtoPWM()
{
  u_int8_t result = analogRead(JOYSTICK_IN);
  Serial.println(result);
  value = adcToDutyCycle(result);
}
// analog to digital converter to duty cycle
double adcToDutyCycle(uint8_t adcValue)
{
  double dutyCycle = (double)adcValue / (double)DAC_RESOLUTION;
  return dutyCycle;
}

// pwm function frequency = 1000 Hz

#define PWM_PERIOD 1000 // 1ms
void *pwm(void *arg)
{
  int high = PWM_PERIOD - (int)(value * PWM_PERIOD);
  int low = PWM_PERIOD - high;
  long long int start_time = micros();
  while (1)
  {
    if (micros() - start_time > high)
    {
      digitalWrite(pin, LOW);
    }
    else
    {
      digitalWrite(pin, HIGH);
    }
    if (micros() - start_time > PWM_PERIOD)
    {
      start_time = micros();
    }
  }
}
