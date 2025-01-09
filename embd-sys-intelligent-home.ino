/*IntelligentHome
  PREPEARED by impedancePlus
  INTELLECTUAL PROPERTY OF impedancePlus
  ALL RIGHTS RESERVED BY impedancePlus
  Initial CODE - VERSION-0.1 -12-06-2022-
  
  COPY RIGHT ©2022
  TEAM & CONTRIBUTORS
  ALVEE (LEAD) | Founder-Owner-CEO | impedancePlus

*/
//*************FEATURES************//
//_____________LIBRARIES___________//
#include<dht.h>
#include <Servo.h>
#include<LiquidCrystal.h>
//_____________VARIABLES___________//

// SERVO PINS
int servoPin = 53; //__UNUSED

//ALARM VARIABLES
//LEDs
byte pin_RedLED = 22;
byte pin_GreenLED = 23;
//Buzzer
byte pin_Buzzer = 24;

//LAMP-LIGHT VARIABLE
byte pin_LAMP = 25;

//FLAME-SENSOR VARIABLES
byte pin_FlameSensor = 15;
byte flameSensorData = 0;

//LDR-SENSOR VARIABLES
byte pin_LDRsensor = 14;
int LDRSensorData;

//MQ-2 GAS/SMOKE SENSOR VARIABLES
byte pin_MQ2Sensor = A7;
int  MQ2SnesorThreshold = 255;
int  MQ2SensorData;

//IR SENSOR
//PIR MOTION SENSOR

//_____________OBJECTS___________//
Servo thisServo;

//_____________FUNCTIONS___________//

//ULTRASONIC'S SERVO __UNUSED
void func_ServoMotor() {
  for (int i = 15; i <= 165; i++) {
    thisServo.write(i);
    delay(10);
  }
  // Repeats the previous lines from 165 to 15 degrees
  for (int i = 165; i >= 15; i--) {
    thisServo.write(i);
    delay(10);
  }
}


//Serial Motion Print For debugging purposes __UNUSED
void func_PrintSerialMonitorSystem() {
  //__UNUSED
}
// Alarm System Function
void func_AlarmSystem(boolean status_ALARM) {
  if (status_ALARM) {
    digitalWrite(pin_RedLED, HIGH);
    digitalWrite(pin_Buzzer, HIGH);
    digitalWrite(pin_GreenLED, LOW);
    delay(205);
    digitalWrite(pin_RedLED, LOW);
    digitalWrite(pin_Buzzer, LOW);
    digitalWrite(pin_GreenLED, LOW);
    delay(205);
  } else if (!status_ALARM) {
    digitalWrite(pin_GreenLED, HIGH);
    digitalWrite(pin_RedLED, LOW);
    digitalWrite(pin_Buzzer, LOW);
    delay(250);
  }

}

//LAMP System Function
void func_LAMPsystem(boolean status_LAMP) {
  if (status_LAMP) {
    digitalWrite(pin_LAMP, HIGH);

  } else if (!status_LAMP) {
    digitalWrite(pin_LAMP, LOW);

  }
}

//Flame Detection Function
void func_FlameDetection() {
 

  flameSensorData = digitalRead(pin_FlameSensor);
  if (flameSensorData == 0) {
    func_AlarmSystem(true);


  } else if (flameSensorData == 1) {
    func_AlarmSystem(false);
  }
}

//LDR Detection Function
void func_LightDetection() {
 
  LDRSensorData = digitalRead(pin_LDRsensor);
  if (LDRSensorData == 1) {
    Serial.println("lampOn");
    func_LAMPsystem(true);


  } else if (LDRSensorData == 0) {
    Serial.println("lampOff");
    func_LAMPsystem(false);
  }
}


//MQ2 Gas-Smoke  Function
void func_GasSmokeDetection() {
 

  MQ2SensorData = analogRead(pin_MQ2Sensor);
  Serial.print("     ");
  Serial.print("mq2 Sensor Data: ");
  Serial.println(MQ2SensorData);
  if (MQ2SensorData > MQ2SnesorThreshold) {
    func_AlarmSystem(true);

  } else if (MQ2SensorData < MQ2SnesorThreshold) {
    func_AlarmSystem(false);
  }
}


//SETUP FUNCTION
void setup() {
  // put your setup code here, to run once:
  //Serial Monitor Initi
  Serial.begin(9600);
  //Leds Pin
  pinMode(pin_RedLED, OUTPUT);
  pinMode(pin_GreenLED, OUTPUT);
  pinMode(pin_Buzzer, OUTPUT);
  //LAMP Leds pin
  pinMode(pin_LAMP, OUTPUT);
  //Flame Sensor Pin
  pinMode(pin_FlameSensor, INPUT);
  //LDR Sensor Pin
  pinMode(pin_LDRsensor, INPUT);
  //MQ-2 Sensor Pin
  pinMode(pin_MQ2Sensor, INPUT);

  //Ultrasonic-servo
  thisServo.attach(servoPin); // Defines on which pin is the servo motor attached

}


//LOOP FUNCTION
void loop() {
  // put your main code here, to run repeatedly:
 
  
  func_FlameDetection();
  func_GasSmokeDetection();
  func_LightDetection();
  //func_ServoMotor();


}
