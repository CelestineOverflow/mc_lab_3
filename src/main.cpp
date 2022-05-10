#include <Arduino.h>
#include <pthread.h>

#define DEBUG 1
// TASK 1
#define DAC_OUTPUT_PIN 13
#define DAC_RESOLUTION 256
#define COMPARATOR_INPUT 13
// TASK 2
#define JOYSTICK_IN 14
#define LED_OUT 13
// prototypes
uint8_t dacProccess();
void *pwmProccess(void *);
String analogReadtoPWM();
double adcToDutyCycle(uint8_t adcValue);
void setupPWM();
// global variables
pthread_t aThread;

void setup()
{
  Serial.begin(115200);
  // TASK 1
  pinMode(COMPARATOR_INPUT, INPUT);
  // TASK 2
  pinMode(LED_OUT, OUTPUT);
  setupPWM();
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
// test array
bool testArray[8] = {false, false, false, false, false, false, false, false};
// TASK 1
uint8_t dacProccess()
{
  uint8_t result = 0x80;
  for (int i = 7; i >= 0; i++)
  {
    analogWrite(DAC_OUTPUT_PIN, result);
    result = digitalRead(COMPARATOR_INPUT) == LOW;
    DEBUG ? true : 0;
    if (result)
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

double value = 0.5;

// read joystick input and output to PWM

String analogReadtoPWM()
{
  u_int8_t result = analogRead(JOYSTICK_IN);
  // THIS MAY BE WRONG WORKING WITH THREADS IS A MESS
  value = adcToDutyCycle(result);
  return String(value);
}
// analog to digital converter to duty cycle
double adcToDutyCycle(uint8_t adcValue)
{
  return (double)adcValue / (double)DAC_RESOLUTION;
}

void setupPWM(){
  pthread_create(&aThread, NULL, pwmProccess, NULL);
}

#define PWM_PERIOD 1000      // 1ms Frequency = 1000 Hz
void *pwmProccess(void *arg) // pwm function
{
  int high = PWM_PERIOD - (int)(value * PWM_PERIOD);
  int low = PWM_PERIOD - high;
  long long int start_time = micros();
  while (1)
  {
    if (micros() - start_time > high)
    {
      digitalWrite(LED_OUT, LOW);
    }
    else
    {
      digitalWrite(LED_OUT, HIGH);
    }
    if (micros() - start_time > PWM_PERIOD)
    {
      start_time = micros();
    }
  }
}
