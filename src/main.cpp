
/**
 * @file main.cpp
 * @author Anthony Côté, Jacob Deniss, Alexandre Falardeau, Justin Houde, Loïc Jolicoeur-Pomerleau, Nathaniel Létourneau, Shawn Miller, Gabriel Rioux
 * @brief Programme du combattant
 * @version 0.2
 * @date 2021-11-01
 *
 * @copyright Copyright (c) 2021
 *
 */
#include <Arduino.h>
#include <LibRobus.h>
#include <Sen0348_Digital/DFRobot_ID809.h>
#include <stdlib.h>
#include <Adafruit_TCS34725.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define RIGHT_WHEEL 1
#define LEFT_WHEEL 0
#define WHEEL_BASE_SPEED 0.25
#define WHEEL_ADD_SPEED 0.05

#define TIMER_LINE_FALLOWER 3
Adafruit_TCS34725 capteur = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X); // création de l'objet capteur

// OBP-704 possible values for suiveur de ligne
#define LINE_CENTER 1.45 // 1 0 0
#define LINE_RIGHT 2.80  // 0 0 1
#define LINE_LEFT 0.79   // 0 1 0

// ANALOGS
#define ANALOG_LINE_FOLLOWER A0

// DIGITALS

// WHEEL
#define ENCODER_STEP 3200
#define WHEEL_DIAMETER 7.62 // cm
#define WHEEL_CIRCUMFERENCE (WHEEL_DIAMETER * PI)
#define RESOLUTION_ENCODER (WHEEL_CIRCUMFERENCE / ENCODER_STEP)
#define ROBOT_CIRCUMFERENCE (19.05 * PI)
#define RIGHT_WHEEL 1
#define LEFT_WHEEL 0

// FOWARD
#define SPEED_MAX_HIGH_DISTANCE 0.6
#define SPEED_MAX_LOW_DISTANCE 0.4
#define SPEED_MIN 0.2
#define ACCELERATION_HIGH_DISTANCE 20  // accelere jusqu'a 10% de la distance
#define DECCELERATION_HIGH_DISTANCE 75 // deccelere a partir de 90% de la distance
#define ACCELERATION_LOW_DISTANCE 20   // accelere jusqu'a 10% de la distance
#define DECCELERATION_LOW_DISTANCE 75  // deccelere a partir de 90% de la distance

// PID
#define KP_FOWARD 0.00330
#define KI_FOWARD 0.000005
#define KP_TURNING 0.00325
#define KI_TURNING 0.000008
uint32_t sumError;

// ENCODER
float readEncoder0;
float readEncoder1;
int distanceEncodeur;

// MOTOR
float motorLeft = 0;
float motorRight = 0;

// SERVOMOTOR
int angleMoteur;

// NRF24L01
RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";
char readDataCarousel[30];
char sendDataCarousel[] = "1";

void NRF24L01_Init();
void NRF24L01_TransmitData(char *data, uint8_t size);
bool turnCarousel();

// step to follow
typedef enum _step
{
  IDENTIFICATION,
  GO_TO_CAROUSEL,
  FIND_GOOD_KEY,
  TAKE_KEY,
  DROP_KEY,
  DRUNK_KEY,
  GO_BACK
} _step;
_step Step;

// HID
typedef enum _identification
{
  MENU,
  DEPOT,
  RETRAIT,
  DEPOT_CLEF,
} _identification;
_identification Identification;

// definition fonction
void executeStep();

// timer function
void timerInit(uint8_t id, void (*func)(), unsigned long delay, int32_t nrep);
void timerDetectionLineFallower();
volatile float voltageValue;

// motor function
void stopMotor();
void movingFowardRobot(uint16_t distance);
void turnedRobot(int angle);
float accelerationDecelerationPID(float pourcentageVitesse, uint8_t acceleration, uint8_t deceleration, float maxSpeed, float speed);

void servoMoteur(int angle);
int detectColor();

void setup()
{
  //BoardInit();
  Serial.begin(9600);
  NRF24L01_Init();

  // timerInit(TIMER_LINE_FALLOWER, timerDetectionLineFallower, 2, -1);

  readEncoder0 = 0;
  readEncoder1 = 0;

  Step = IDENTIFICATION;

  // SOFT_TIMER_Enable(TIMER_LINE_FALLOWER); //active suiveur de ligne
}

void loop()
{
  // SOFT_TIMER_Update();

  bool value = turnCarousel();
  Serial.println(value);

  if (value)
    Serial.println("fonctionne!!!");
  else
    Serial.println("fonctionne pas...");

  delay(1000);
  // executeStep();
}

void executeStep()
{

  switch (Step)
  {
  case IDENTIFICATION:

    Step = GO_TO_CAROUSEL;
    break;

  case GO_TO_CAROUSEL:

    Step = FIND_GOOD_KEY;
    break;

  case FIND_GOOD_KEY:

    // Step = TAKE_KEY;
    // Step = DROP_KEY;
    // Step = DRUNK_KEY;
    break;

  case TAKE_KEY:

    Step = GO_BACK;
    break;

  case DROP_KEY:

    Step = GO_BACK;
    break;

  case DRUNK_KEY:

    Step = GO_BACK;
    break;

  case GO_BACK:

    Step = IDENTIFICATION;
    break;
  }
}

void timerInit(uint8_t id, void (*func)(), unsigned long delay, int32_t nrep)
{

  SOFT_TIMER_SetCallback(id, func);
  SOFT_TIMER_SetDelay(id, delay);
  SOFT_TIMER_SetRepetition(id, nrep);
}

void timerDetectionLineFallower()
{
  voltageValue = (analogRead(ANALOG_LINE_FOLLOWER)) * (5 / 1023.0);
  // Serial.println(voltageValue);
}

void stopMotor()
{
  MOTOR_SetSpeed(RIGHT_WHEEL, 0);
  MOTOR_SetSpeed(LEFT_WHEEL, 0);
}

void turnedRobot(int angle)
{

  uint8_t angleTmp;
  if (angle < 0)
    angleTmp = angle * -1;
  else
    angleTmp = angle;

  distanceEncodeur = (((ROBOT_CIRCUMFERENCE * angleTmp) / 360) / RESOLUTION_ENCODER);

  readEncoder0 = 0;
  readEncoder1 = 0;
  motorLeft = 0;
  motorRight = 0;

  ENCODER_Reset(RIGHT_WHEEL);
  ENCODER_Reset(LEFT_WHEEL);

  while (((distanceEncodeur) > abs(readEncoder0)) && ((distanceEncodeur) > abs(readEncoder1)))
  { // les 2 roue tournes

    motorLeft = SPEED_MIN;

    // PID FOR ENCODER
    float error = abs(readEncoder0) - abs(readEncoder1);
    float p = error * KP_TURNING;

    sumError += error;
    float i = sumError * KI_FOWARD;

    float adjSpeed = p + i;
    motorRight = motorLeft + adjSpeed;

    if (angle > 0)
    { // test si l'angle est negative, tourne vers la droite
      motorLeft = -1 * motorLeft;
    }
    else if (angle < 0)
    { // test si l'angle est positive, tourne vers la gauche
      motorRight = -1 * motorRight;
    }

    MOTOR_SetSpeed(LEFT_WHEEL, motorLeft);
    MOTOR_SetSpeed(RIGHT_WHEEL, motorRight);

    readEncoder0 = ENCODER_Read(LEFT_WHEEL);
    readEncoder1 = ENCODER_Read(RIGHT_WHEEL);
  }

  stopMotor();

  /*Serial.println(angle);
  Serial.println(distanceEncodeur);
  Serial.println(readEncoder0);
  Serial.println(readEncoder1);
  Serial.println(" ");*/
}

void movingFowardRobot(uint16_t distance)
{
  // uint16_t distance (CENTIMÈTRE);
  // bool sens (0 = à l'envers. 1 = à l'endroit);
  distanceEncodeur = (distance / RESOLUTION_ENCODER);
  readEncoder0 = 0;
  readEncoder1 = 0;
  motorLeft = 0;
  motorRight = 0;
  sumError = 0;

  ENCODER_Reset(RIGHT_WHEEL);
  ENCODER_Reset(LEFT_WHEEL);

  while (((distanceEncodeur) > readEncoder0) && ((distanceEncodeur) > readEncoder1))
  {

    readEncoder0 = ENCODER_Read(LEFT_WHEEL);
    readEncoder1 = ENCODER_Read(RIGHT_WHEEL);

    float pourcentageVitesse = float(readEncoder0 / distanceEncodeur) * 100;

    if (distance > 80)
      motorLeft = accelerationDecelerationPID(pourcentageVitesse, ACCELERATION_HIGH_DISTANCE, DECCELERATION_HIGH_DISTANCE, SPEED_MAX_HIGH_DISTANCE, motorLeft);
    else
      motorLeft = accelerationDecelerationPID(pourcentageVitesse, ACCELERATION_LOW_DISTANCE, DECCELERATION_LOW_DISTANCE, SPEED_MAX_LOW_DISTANCE, motorLeft);

    // PID FOR ENCODER
    float error = readEncoder0 - readEncoder1;
    float p = error * KP_FOWARD;

    sumError += error;
    float i = sumError * KI_FOWARD;

    float adjSpeed = p + i;
    motorRight = motorLeft + adjSpeed;

    MOTOR_SetSpeed(LEFT_WHEEL, motorLeft);
    MOTOR_SetSpeed(RIGHT_WHEEL, motorRight);
  }

  stopMotor();

  /*Serial.println(distance);
  Serial.println(distanceEncodeur);
  Serial.println(readEncoder0);
  Serial.println(readEncoder1);
  Serial.println(" ");*/
}

float accelerationDecelerationPID(float pourcentageVitesse, uint8_t acceleration, uint8_t deceleration, float maxSpeed, float speed)
{

  float _speed = 0;
  float dAcceleration = maxSpeed / acceleration;
  float dDeceleration = -maxSpeed / (100 - deceleration);
  float b = maxSpeed - (dDeceleration * deceleration); // pente de la fonction de decrementation

  if (acceleration >= pourcentageVitesse)
  { // de 0 a ACCELERATION%
    _speed = pourcentageVitesse * dAcceleration;

    if (_speed < SPEED_MIN)
      _speed = SPEED_MIN;
    if (_speed >= maxSpeed)
      _speed = maxSpeed;

    return _speed;
  }
  else if (deceleration <= pourcentageVitesse)
  { // de DECELERATION a 100%
    _speed = dDeceleration * pourcentageVitesse + b;

    if (_speed < SPEED_MIN)
      _speed = SPEED_MIN;

    return _speed;
  }
  else
  {
    return speed;
  }
}

int detectColor()
{
  uint16_t clear, red, green, blue;
  char couleur[1];
  capteur.begin();
  delay(250); // takes 50ms to read
  capteur.getRawData(&red, &green, &blue, &clear);

  if (red > 500 && red < 600 && green > 525 && green < 625 && blue > 310 && blue < 410)
  {
    couleur[0] = 'j'; // test
    Serial.print(" \ncouleur\t ");
    Serial.print(couleur); // test
    return 1;
  }
  else if (red > 450 && red < 550 && green > 375 && green < 475 && blue > 320 && blue < 430)
  {
    couleur[0] = 'r'; // test
    Serial.print(" \ncouleur\t ");
    Serial.print(couleur); // test
    return 2;
  }
  else if (red > 350 && red < 450 && green > 460 && green < 560 && blue > 370 && blue < 470)
  {
    couleur[0] = 'b'; // test
    Serial.print(" \ncouleur\t ");
    Serial.print(couleur); // test
    return 3;
  }
  Serial.print("C:\t");
  Serial.print(clear);
  Serial.print("\tR:\t");
  Serial.print(red);
  Serial.print("\tG:\t");
  Serial.print(green);
  Serial.print("\tB:");
  Serial.print(blue);
  Serial.println(" ");
  return 0;
}

void servoMoteur(int angle)
{
  if (angle > angleMoteur)
  {
    for (int i = angleMoteur; i < angle; i += 1)
    {
      SERVO_SetAngle(0, i);
      SERVO_SetAngle(1, i);
      delay(15);
    }
  }
  else
  {
    for (int i = angleMoteur; i > angle; i -= 1)
    {
      SERVO_SetAngle(0, i);
      SERVO_SetAngle(1, i);
      delay(15);
    }
  }
  angleMoteur = angle;
}

void NRF24L01_Init()
{
  radio.begin();
  // receiver
  radio.openReadingPipe(0, address);

  // transmitter
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
}

void NRF24L01_TransmitData(char *data, uint8_t size)
{
  radio.stopListening();

  radio.write(data, size);
  //Serial.println(data);

  radio.startListening();
}

bool turnCarousel()
{

  NRF24L01_TransmitData(sendDataCarousel, sizeof(sendDataCarousel));

  int cpt = 0;
  while (!radio.available()){ // attent le retour de commande carousel
    delay(1);
    cpt++;
    if(cpt >= 1000)
      return 0;
  } 

  radio.read(&readDataCarousel, 1);
  Serial.println(readDataCarousel);
  radio.flush_rx();

  if (readDataCarousel[0] == '1')
    return 1;
  else
    return 0;
}
