#include <Arduino.h>
#include <pthread.h>
#include <esp_task_wdt.h>

#define DEBUG 1
#define BAUDRATE 115200
// TASK 1
#define DAC_OUTPUT_PIN 13
#define VOLTAGE_STEP 0.01953125;
#define DAC_RESOLUTION 256
#define COMPARATOR_INPUT 13
// TASK 2
#define JOYSTICK_IN 14
#define LED_OUT 13
//BCD PORTS
#define BCD_PORT_D0 0
#define BCD_PORT_D1 1
#define BCD_PORT_D2 2
#define BCD_PORT_D3 3
#define BCD_PORT_D4 4
#define BCD_PORT_D5 5
#define BCD_PORT_D6 6
#define BCD_PORT_D7 7
//enable pins
#define ENABLE_T0 8
#define ENABLE_T1 9
// prototypes
uint8_t dacProccess();
void *pwmProccess(void *);
String analogReadtoPWM();
double adcToDutyCycle(uint8_t adcValue);
void setupPWM();
void changeValue(double value);
// global variables
pthread_t aThread;
pthread_mutex_t lock;

void setup()
{
  Serial.begin(BAUDRATE);
  Serial.println("Starting");
  // TASK 1
  pinMode(COMPARATOR_INPUT, INPUT);
  // TASK 2
  pinMode(LED_OUT, OUTPUT);
  setupPWM();
}

long long int last_time = 0;
double value = 0.5;

void loop()
{
  /*Serial.println("DAC value: " + String(dacProccess()));
  delay(1000);*/
  if (millis() - last_time > 1000)
  {
    // Serial.println("DAC value: " + String(dacProccess()));
    // analogReadtoPWM();
    pthread_mutex_lock(&lock);
    value += 0.1;
    if (value > 0.95)
    {
      value = 0.1;
    }
    Serial.println(value);
    changeValue(value);
    pthread_mutex_unlock(&lock);
    last_time = millis();
  }
}
// test array
bool testArray[8] = {false, false, true, false, false, false, false, false};
// TASK 1
uint8_t dacProccess()
{
  uint8_t result = 0x80;
  for (int i = 7; i >= 0; i--)
  {
    // analogWrite(DAC_OUTPUT_PIN, result);
    // comparatorResult = digitalRead(COMPARATOR_INPUT) == LOW;
    Serial.println(i);
    if (testArray[i])
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

#define PWM_PERIOD 1000 // 1ms Frequency = 1000 Hz
int high, low;
void setupPWM()
{
  pthread_create(&aThread, NULL, pwmProccess, NULL);
}

void changeValue(double value)
{
  high = PWM_PERIOD - (int)(value * PWM_PERIOD);
  low = PWM_PERIOD - high;
  Serial.println(high);
  Serial.println(low);
}

void *pwmProccess(void *arg) // pwm function
{
  esp_task_wdt_init(1000, false);
  pthread_mutex_lock(&lock);
  long long int start_time = micros();
  while (1)
  {
    if (micros() - start_time > high)
    {
      digitalWrite(LED_OUT, HIGH);
    }
    else
    {
      digitalWrite(LED_OUT, LOW);
    }
    if (micros() - start_time > PWM_PERIOD)
    {
      start_time = micros();
    }
  }
}

typedef enum {section_0, section_1} section_t;

void proccess(uint8_t dac_input){
  String kk = convertshit(dac_input);
  Serial.println(kk);
  int digit0 = kk.charAt(0) - '0';
  int digit1 = kk.charAt(1) - '0';
  int digit2 = kk.charAt(2) - '0';
  uint8_t Array[4] = {0,digit2,digit1,digit0};
  BCD_Displayh(Array, section_0);
  BCD_Displayh(Array, section_1);
}

String convertshit(uint8_t dac_input){
  double voltage = (double)dac_input * VOLTAGE_STEP;
  int voltageInteger = (int)(voltage * 100);
  String voltageString = String(voltageInteger);
  if (voltageString.length() < 3)
  {
    voltageString = "0" + voltageString;
  }
  if (voltageString.length() < 2)
  {
    voltageString = "00" + voltageString;
  }
  return voltageString;
}

void BCD_Displayh(uint8_t *inputArray, section_t section)
{
  digitalWrite(ENABLE_T0, LOW);
  digitalWrite(ENABLE_T1, LOW);
  switch (section)
  {
    case section_0:
      digitalWrite(ENABLE_T0, HIGH);
      writetoport(inputArray[0], inputArray[1]);
      break;
    case section_1:
      digitalWrite(ENABLE_T1, HIGH);
      writetoport(inputArray[2], inputArray[3]);
      break;
    default:
      break;
  }
}

void writetoport(uint8_t input0, uint8_t input1){
  //first digit
  digitalWrite(BCD_PORT_D0, input0 & (1 << 0));
  digitalWrite(BCD_PORT_D1, input0 & (1 << 1));
  digitalWrite(BCD_PORT_D2, input0 & (1 << 2));
  digitalWrite(BCD_PORT_D3, input0 & (1 << 3));
  //second digit
  digitalWrite(BCD_PORT_D4, input1 & (1 << 4));
  digitalWrite(BCD_PORT_D5, input1 & (1 << 5));
  digitalWrite(BCD_PORT_D6, input1 & (1 << 6));
  digitalWrite(BCD_PORT_D7, input1 & (1 << 7));
}