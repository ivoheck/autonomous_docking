//https://dronebotworkshop.com/stepper-motors-with-arduino/
//https://tutorials-raspberrypi.de/arduino-raspberry-pi-miteinander-kommunizieren-lassen/

#include <AccelStepper.h>

//Links Hinten
#define STEP_PIN_1 26
#define DIR_PIN_1 28
#define ENABLE_PIN_1 24

//Links Vorne
#define STEP_PIN_2 36
#define DIR_PIN_2 34
#define ENABLE_PIN_2 30

//Rechts Vorne
#define STEP_PIN_3 46
#define DIR_PIN_3 48
#define ENABLE_PIN_3 62

//Rechts Hinten
#define STEP_PIN_4 60
#define DIR_PIN_4 61
#define ENABLE_PIN_4 56

AccelStepper stepper1(AccelStepper::DRIVER, STEP_PIN_1, DIR_PIN_1);
AccelStepper stepper2(AccelStepper::DRIVER, STEP_PIN_2, DIR_PIN_2);
AccelStepper stepper3(AccelStepper::DRIVER, STEP_PIN_3, DIR_PIN_3);
AccelStepper stepper4(AccelStepper::DRIVER, STEP_PIN_4, DIR_PIN_4);

int speeds[4];
int maxSpeed = 5036;
int multiplyer = maxSpeed/100;

int convert_speed(int speed){
  return (int)(speed * multiplyer);
}

void setup() {
  Serial.begin(9600);
  
  stepper1.setMaxSpeed(maxSpeed); // Schritte pro Sekunde
  stepper2.setMaxSpeed(maxSpeed);
  stepper3.setMaxSpeed(maxSpeed); 
  stepper4.setMaxSpeed(maxSpeed);   

  stepper1.setSpeed(0);     // Schritte pro Sekunde
  stepper2.setSpeed(0);
  stepper3.setSpeed(0);      
  stepper4.setSpeed(0); 
}

void loop() {
  pinMode(ENABLE_PIN_1, OUTPUT);
  pinMode(ENABLE_PIN_2, OUTPUT); 
  pinMode(ENABLE_PIN_3, OUTPUT); 
  pinMode(ENABLE_PIN_4, OUTPUT); 
  
  digitalWrite(ENABLE_PIN_1, LOW);
  digitalWrite(ENABLE_PIN_2, LOW);
  digitalWrite(ENABLE_PIN_3, LOW);
  digitalWrite(ENABLE_PIN_4, LOW);
  
  stepper1.setSpeed(speeds[0]);
  stepper2.setSpeed(speeds[1]);
  stepper3.setSpeed(-speeds[2]); // auf der rechten seite wird die richtung umgedreht
  stepper4.setSpeed(-speeds[3]);
  
  if (Serial.available() >= 16) {
    for(int i = 0;i<4; i++){
        char inChar = (char)Serial.read();
        bool positiv = true;
        
        if (inChar == '-') {
          positiv = false;
        }
        
        int speed = 0;

    for(int j = 100; j >= 1; j /= 10){
        char inChar = (char)Serial.read();
        int diget = inChar -'0';

        if(diget == 1){
          speeds[i] = diget * j;
        }
        
        speed += diget * j;
      }
    speed = convert_speed(speed);
    speeds[i] = positiv ? speed : -speed;
    }
    
  }


  stepper1.runSpeed();
  stepper2.runSpeed();
  stepper3.runSpeed();
  stepper4.runSpeed();
}
