#include <Arduino_FreeRTOS.h>
#include <Servo.h>
#include <semphr.h>
#include "queue.h"

QueueHandle_t var_queue;
SemaphoreHandle_t xBinarySemaphore;

#define IRSensor 2   // connect IR sensor to Arduino Pin 2
#define PIRSensor 7  // connect PIR sensor to Arduino Pin 7
#define Red_LED 13   // conect Red_LED to Arduino Pin 13
#define ServoPin 3
#define Button 4

Servo MyServo;

int i = 0;

void setup() {

  Serial.begin(9600);
  // put your setup code here, to run once:
  var_queue = xQueueCreate(1, sizeof(unsigned int));
  xBinarySemaphore = xSemaphoreCreateBinary();

  xTaskCreate(Task1, "IR_Sensor", 128, NULL, 3, NULL);
  xTaskCreate(Task2, "ServoMotor", 128, NULL, 2, NULL);
  xTaskCreate(Task3, "PIRSensor", 128, NULL, 1, NULL);

  xSemaphoreGive(xBinarySemaphore);
}

void Task1(void *pvparameters) {

  pinMode(IRSensor, INPUT);  // sensor pin INPUT
  pinMode(Red_LED, OUTPUT);  // Red_Led pin OUTPUT

  while (1) {

    int statusSensor = digitalRead(IRSensor);

    if (statusSensor == 1) {

      int Status_Send = 0;
      xQueueSend(var_queue, &Status_Send, 2);
      Serial.println("Vehicle is not there");
      vTaskDelay(1);
    }

    else {

      int Status_Send = 1;
      xQueueSend(var_queue, &Status_Send, 2);
      digitalWrite(Red_LED, HIGH);  // Red_LED High
      Serial.println("Vehicle is there");
      Serial.println("Red LED is ON");
      Serial.println("");
      vTaskDelay(1);
    }
  }
}

void Task2(void *pvparameters) {

  pinMode(Button, INPUT);

  MyServo.attach(ServoPin);

  int32_t StatusReceived = 0;
  int32_t button_state = 0;

  const TickType_t xTicksToWait = pdMS_TO_TICKS(100);

  while (1) {

    xQueueReceive(var_queue, &StatusReceived, xTicksToWait);
    vTaskDelay(1);
    Serial.print("Status received = ");
    Serial.println(StatusReceived);
    vTaskDelay(1);

    if (StatusReceived == 1) {

      MyServo.write(0);
      Serial.println("Gate Closed");
      Serial.println("");
      vTaskDelay(1);

      delay(100);
    }

    button_state = digitalRead(Button);
    int Gate_Opened = 0;

    if (button_state == HIGH) {

      MyServo.write(90);
      int Gate_Opened = 1;
      xQueueSend(var_queue, &Gate_Opened, 2);
      digitalWrite(Red_LED, LOW);  // Red_LED LOW
      Serial.println("Button is pressed");
      Serial.println("Gate Opened");
      Serial.println("Red LED Turns OFF");

      vTaskDelay(10);
    }
  }
}

void Task3(void *pvparameters) {

  int PIRState = LOW;  // we start, assuming no motion detected
  int val = 0;         // variable for reading the pin status
  int counter = 0;
  int CurrentState = 0;
  int PreviousState = 0;

  const TickType_t xTicksToWait = pdMS_TO_TICKS(100);

  pinMode(PIRSensor, INPUT);  // declare PIR Sensor as input

  int32_t GateOpenReceived = 0;


  while (1) {

    xQueueReceive(var_queue, &GateOpenReceived, xTicksToWait);
    Serial.print("GateOpenReceived = ");
    Serial.println(GateOpenReceived);
    vTaskDelay(1);

    if (GateOpenReceived == 1) {

      xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);

      val = digitalRead(PIRSensor);  // read PIR Sensor input value
      if (val == HIGH) {             // check if the input is HIGH
        Serial.println("PIR Input is HIGH");
        vTaskDelay(1);
        if (PIRState == LOW) {
          // we have just turned on
          CurrentState = 1;
          // We only want to print on the output change, not state
          PIRState = HIGH;
          vTaskDelay(10);
        }
      }

      else {
        Serial.println("PIR Input is LOW");
        vTaskDelay(1);
        if (PIRState == HIGH) {
          // we have just turned of
          CurrentState = 0;
          // We only want to print on the output change, not state
          PIRState = LOW;
        }
      }

      if (CurrentState != PreviousState) {
        if (CurrentState == 1) {
          counter = counter + 1;
          Serial.println("");
          Serial.println("");
          Serial.print("Number of Vehicles = ");
          Serial.println(counter);
          Serial.println("");
          Serial.println("");
          vTaskDelay(100);
        }
      }

      xSemaphoreGive(xBinarySemaphore);
      vTaskDelay(1);
    }
  }
}

void loop() {
  // put your main code here, to run repeatedly:
}
