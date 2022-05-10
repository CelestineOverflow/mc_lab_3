#include <Arduino.h>
#include <pthread.h>
#define DAC_PIN A0
#define DAC_RESOLUTION 256
#define DAC_INPUT_PIN 13
#define JOYSTICK_IN 14
#define LED_OUT 13
// prototypes
uint8_t dacProccess();

void setup()
{
  Serial.begin(115200);
  pinMode(DAC_PIN, OUTPUT);
  pinMode(LED_OUT, OUTPUT);
  // pinMode(DAC_INPUT_PIN, INPUT);
  // pinMode(JOYSTICK_IN, INPUT);
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

uint8_t dacProccess()
{
  uint8_t result = 0x80;
  for (int i = 7; i >= 0; i++)
  {
    analogWrite(DAC_PIN, result);
    if (digitalRead(DAC_INPUT_PIN) == LOW)
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
double value = 0;
// pwm function

pthread_t aThread;

// read joystick input and output to PWM

void analogReadtoPWM()
{
  u_int8_t result = analogRead(JOYSTICK_IN);
  Serial.println(result);
  pthread_cancel(aThread);
  value = adcToDutyCycle(result);
  pthread_create(&aThread, NULL, pwm, NULL);
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
