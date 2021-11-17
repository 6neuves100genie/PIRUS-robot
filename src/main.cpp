
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

#define RIGHT_WHEEL 1
#define LEFT_WHEEL 0
#define WHEEL_BASE_SPEED 0.25
#define WHEEL_ADD_SPEED 0.05


<<<<<<< HEAD
#define TIMER_LINE_FALLOWER 3
Adafruit_TCS34725 capteur = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X); // création de l'objet capteur
=======
// OBP-704 possible values for suiveur de ligne
#define LINE_LEFT 1.45   
#define LINE_RIGHT 2.80   
#define INTERSECTION 0.79
>>>>>>> 7a4ddc1d12666e5726164297885eeeac711feb49

// OBP-704 possible values for suiveur de ligne
#define LINE_CENTER 1.45   // 1 0 0
#define LINE_RIGHT 2.80  // 0 0 1
#define LINE_LEFT 0.79 // 0 1 0

// ANALOGS
#define ANALOG_LINE_FOLLOWER A0

// DIGITALS


//WHEEL
#define ENCODER_STEP 3200
#define WHEEL_DIAMETER 7.62 //cm
#define WHEEL_CIRCUMFERENCE (WHEEL_DIAMETER * PI)
#define RESOLUTION_ENCODER (WHEEL_CIRCUMFERENCE / ENCODER_STEP)
#define ROBOT_CIRCUMFERENCE (19.05 * PI)
#define RIGHT_WHEEL 1
#define LEFT_WHEEL 0

//FOWARD
#define SPEED_MAX_HIGH_DISTANCE 0.6
#define SPEED_MAX_LOW_DISTANCE 0.4
#define SPEED_MIN 0.2
#define ACCELERATION_HIGH_DISTANCE 20  //accelere jusqu'a 10% de la distance
#define DECCELERATION_HIGH_DISTANCE 75 //deccelere a partir de 90% de la distance
#define ACCELERATION_LOW_DISTANCE 20   //accelere jusqu'a 10% de la distance
#define DECCELERATION_LOW_DISTANCE 75  //deccelere a partir de 90% de la distance

//PID
#define KP_FOWARD 0.00330
#define KI_FOWARD 0.000005
#define KP_TURNING 0.00325
#define KI_TURNING 0.000008
uint32_t sumError;

// ENCODER
float readEncoder0;
float readEncoder1;
int distanceEncodeur;

//MOTOR
float motorLeft = 0;
float motorRight = 0;

<<<<<<< HEAD
int angleMoteur;

typedef enum _step
{

} _step;
_step Step;

//definition fonction

void executeStep();

//timer function
void timerInit(uint8_t id, void (*func)(), unsigned long delay, int32_t nrep);
void timerDetectionLineFallower();
volatile float voltageValue;

//motor function
=======
//TIMER0
volatile uint8_t compt = 0;
volatile bool encoderEqual = false;

//ETAPE PARCOURS
#define NBR_DISTANCE 2
#define NBR_DISTANCEr 1
#define NBR_DISTANCErr 2
#define NBR_DISTANCEj 3
#define NBR_DISTANCEjr 4
#define NBR_DISTANCEb 3
#define NBR_DISTANCEbr 4
#define NBR_ANGLE 2
#define NBR_ANGLEr 1
#define NBR_ANGLErr 2
#define NBR_ANGLEj 3
#define NBR_ANGLEjr 4
#define NBR_ANGLEb 3
#define NBR_ANGLEbr 4
float tabDistance[NBR_DISTANCE] = {0, 45};
float tabDistancer[NBR_DISTANCEr] = {215};
float tabDistancerr[NBR_DISTANCErr] = {0, 215};
float tabDistancej[NBR_DISTANCEj] = {0, 40, 215};
float tabDistancejr[NBR_DISTANCEjr] = {0, 215, 40, 75};
float tabDistanceb[NBR_DISTANCEb] = {0, 40, 215};
float tabDistancebr[NBR_DISTANCEbr] = {0, 215, 40, 75};
int tabAngle[NBR_ANGLE] = {185, 0};
int tabAngler[NBR_ANGLEr] = {0};
int tabAnglerr[NBR_ANGLErr] = {-180, 0};
int tabAnglej[NBR_ANGLEj] = {-89, 91, 0};
int tabAnglejr[NBR_ANGLEjr] = {-182, -90, 90, 0};
int tabAngleb[NBR_ANGLEb] = {89, -91, 0};
int tabAnglebr[NBR_ANGLEbr] = {-180, 90, -90, 0};
int tmpValueTab;
bool etapeCouleur = false;

bool executionStepParcours(uint16_t distance, int angle);
>>>>>>> 7a4ddc1d12666e5726164297885eeeac711feb49
void stopMotor();
void movingFowardRobot(uint16_t distance);
void turnedRobot(int angle);
float accelerationDecelerationPID(float pourcentageVitesse, uint8_t acceleration, uint8_t deceleration, float maxSpeed, float speed);

void servoMoteur(int angle);
<<<<<<< HEAD
int detectColor();

=======
void init_Detection5kHz();
void detectSound();
void lightLED(bool red, bool green, bool blue, bool yellow);
void initLEDsPin();
void turnToBeInLine(bool sens);
>>>>>>> 7a4ddc1d12666e5726164297885eeeac711feb49

void setup()
{
  BoardInit();

  delay(2000);

  timerInit(TIMER_LINE_FALLOWER, timerDetectionLineFallower, 2, -1);


  readEncoder0 = 0;
  readEncoder1 = 0;
<<<<<<< HEAD
  //servoMoteur(150);

  //Step = FALLOW_LINE1;

  SOFT_TIMER_Enable(TIMER_LINE_FALLOWER); //active suiveur de ligne
=======
  SOFT_TIMER_SetCallback(FOLLOW_LINE_TIMER, followLine);
  SOFT_TIMER_SetCallback(DETECT_BOWLING_PIN_TIMER, detectBowlingPin);
  SOFT_TIMER_SetCallback(DETECT_SOUND_TIMER, detectSound);

  //Ajout du temps de répétition
  //SOFT_TIMER_SetDelay(int id, int ms)
  SOFT_TIMER_SetDelay(FOLLOW_LINE_TIMER, 2);
  SOFT_TIMER_SetDelay(DETECT_BOWLING_PIN_TIMER, 150);
  SOFT_TIMER_SetDelay(DETECT_SOUND_TIMER, 20);

  //Ajout du nombre de fois qu'il répétera la fonction (-1 = infini);
  //SOFT_TIMER_SetRepetition(int id, int nbFois)
  SOFT_TIMER_SetRepetition(FOLLOW_LINE_TIMER, -1);
  SOFT_TIMER_SetRepetition(DETECT_BOWLING_PIN_TIMER, -1);
  SOFT_TIMER_SetRepetition(DETECT_SOUND_TIMER, -1);

  //détermine si le timer pour la fonction et activé ou désactivé.
  //SOFT_TIMER_Enable(int id);
  //SOFT_TIMER_disable(int id);
  SOFT_TIMER_Enable(FOLLOW_LINE_TIMER);
  SOFT_TIMER_Enable(DETECT_BOWLING_PIN_TIMER);
  //SOFT_TIMER_Enable(DETECT_SOUND_TIMER);
  
  lightLED(1, 0, 0, 0);
  delay(500);
  lightLED(1, 0, 1, 0);
  delay(500);
  lightLED(1, 0, 1, 1);
>>>>>>> 7a4ddc1d12666e5726164297885eeeac711feb49

}

void loop()
{
<<<<<<< HEAD
  SOFT_TIMER_Update();

  executeStep();
=======
 porterBalle();
 // SOFT_TIMER_Update();
>>>>>>> 7a4ddc1d12666e5726164297885eeeac711feb49
}

void executeStep()
{
<<<<<<< HEAD

  /*switch (Step)
=======
  // Ajustement des roues pour le suiveur de ligne
  // Vue qu'on se base sur les capteurs, nous n'utilisons pas de PID :)
  //on va chercher le voltage
  //on ajuste les roues dépendant du voltage
  float voltageValue = (analogRead(ANALOG_LINE_FOLLOWER)) * (5 / 1023.0);
  Serial.println(voltageValue);
  float motor_left = 0;
  float motor_right = 0;
  float motor_base_value = 0.5;
 /* if (etapeCouleur && ((
    (voltageValue >= LINE_COLOUR - 0.05 && voltageValue <= LINE_COLOUR + 0.05) ||
    (voltageValue >= LINE_COLOUR2 - 0.05 && voltageValue <= LINE_COLOUR2 + 0.05) ||
    (voltageValue >= LINE_COLOUR3 - 0.05 && voltageValue <= LINE_COLOUR3 + 0.05) ||
    (voltageValue >= LINE_COLOUR4 - 0.05 && voltageValue <= LINE_COLOUR4 + 0.05)))){
      SOFT_TIMER_Disable(FOLLOW_LINE_TIMER);
      stopMotor();
      delay(500);
      turnedRobot(45);
      turnToBeInLine(true);

  }*/
  /*else*/ if ((voltageValue >= LINE_RIGHT - 0.05 && voltageValue <= LINE_RIGHT + 0.05) || (voltageValue >= LINE_UNDECIDED - 0.05 && voltageValue <= LINE_UNDECIDED + 0.05))
  {
    motor_right = motor_base_value;
    motor_left = -motor_base_value;
  }
  else if (voltageValue >= LINE_LEFT - 0.05 && voltageValue <= LINE_LEFT + 0.05)
>>>>>>> 7a4ddc1d12666e5726164297885eeeac711feb49
  {
  case FALLOW_LINE1:

    break;

<<<<<<< HEAD
  case FIND_PIN:
=======
void detectBowlingPin()
{
  //Mesurer avec l'infrarouge ou le sonar
  float distance = SONAR_GetRange(1);
  float voltageValue = (analogRead(ANALOG_LINE_FOLLOWER)) * (5 / 1023.0);
  //Serial.println(distance);
  if (distance < 50.00 && distance > 0)
  {
    SOFT_TIMER_Disable(FOLLOW_LINE_TIMER);
    stopMotor();
    lightLED(1, 1, 1, 1);

    turnedRobot(-90);
    delay(500);
    SOFT_TIMER_Enable(FOLLOW_LINE_TIMER);

    //findLineAgain();
    //tourner vers la droite
    //CONTINUER D'AVANCER JUSQU'À
    //V
    //retrouve la linge
    //recommence FOLLOW_LINE_TIMER
    //TROUVER INTERSECTION
    //DONNE LE CODE A ALEX
    if(voltageValue >= INTERSECTION - 0.05 && voltageValue <= INTERSECTION + 0.05){
      turnedRobot(90);
    }
    SOFT_TIMER_Disable(DETECT_BOWLING_PIN_TIMER);
  }
  //Prend en note à chaques fois qu'elle le détecte
}
>>>>>>> 7a4ddc1d12666e5726164297885eeeac711feb49

    break;

  case TURN_TO_PIN:

    break;

  case GO_TO_PIN:

    break;

  case TURN_RIGHT:

    break;

<<<<<<< HEAD
  case FALLOW_LINE_COLOR:

    break;

  case TURN_LEFT:

    break;
=======
  float voltageValue = 0;
  bool leftFound = false;
  bool rightFound = false;
  MOTOR_SetSpeed(LEFT_WHEEL, 200*modif);
  MOTOR_SetSpeed(RIGHT_WHEEL, -200*modif);
  while (!(leftFound && rightFound))
  {
    voltageValue = analogRead(ANALOG_LINE_FOLLOWER) * (5 / 1023.0);
    if (voltageValue >= LINE_RIGHT - 0.05 && voltageValue <= LINE_RIGHT + 0.05)
    {
      leftFound = true;
    }
    else if (voltageValue >= LINE_LEFT - 0.05 && voltageValue <= LINE_LEFT + 0.05)
    {
      rightFound = true;
    }
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
    lightLED(0, 0, 0, 1);
    couleur[0] = 'j'; //test
    Serial.print(" \ncouleur\t ");
    Serial.print(couleur); //test
    return 1;
  }
  else if (red > 450 && red < 550 && green > 375 && green < 475 && blue > 320 && blue < 430)
  {
    lightLED(1, 0, 0, 0);
    couleur[0] = 'r'; //test
    Serial.print(" \ncouleur\t ");
    Serial.print(couleur); //test
    return 2;
  }
  else if (red > 350 && red < 450 && green > 460 && green < 560 && blue > 370 && blue < 470)
  {
    lightLED(0, 0, 1, 0);
    couleur[0] = 'b'; //test
    Serial.print(" \ncouleur\t ");
    Serial.print(couleur); //test
    return 3;
  }
  Serial.print("C:\t"); Serial.print(clear);
    Serial.print("\tR:\t"); Serial.print(red);
    Serial.print("\tG:\t"); Serial.print(green);
    Serial.print("\tB:"); Serial.print(blue);
	Serial.println(" ");
  return 0;
}

void porterBalle()
{
servoMoteur(150);
int couleur=detectColor();
if(couleur==0){
  delay(500);
 couleur= detectColor();}
 if(couleur==0){
   delay(500);
 couleur= detectColor();}

 if(couleur==0){
   delay(500);
 couleur= detectColor();}
  delay(2000);
  for (int i = 0; NBR_DISTANCE > i; i++)
  {
    executionStepParcours(tabDistance[i], tabAngle[i]);
  }
  delay(1500);
  servoMoteur(100);
  delay(1500);
  if (couleur == 1)
  {
    for (int i = 0; NBR_DISTANCEj > i; i++)
    {
      executionStepParcours(tabDistancej[i], tabAnglej[i]);
    }
    delay(2500);
    servoMoteur(150);
    delay(1500);
    for (int i = 0; NBR_DISTANCEjr > i; i++)
    {
      executionStepParcours(tabDistancejr[i], tabAnglejr[i]);
    }
  }
  if (couleur == 2)
  {
    for (int i = 0; NBR_DISTANCEr > i; i++)
    {
      executionStepParcours(tabDistancer[i], tabAngler[i]);
    }
    delay(2500);
    servoMoteur(150);
    delay(1500);
    for (int i = 0; NBR_DISTANCErr > i; i++)
    {
      executionStepParcours(tabDistancerr[i], tabAnglerr[i]);
    }
  }
  if (couleur == 3)
  {
    for (int i = 0; NBR_DISTANCEb > i; i++)
    {
      executionStepParcours(tabDistanceb[i], tabAngleb[i]);
    }
    delay(2500);
    servoMoteur(150);
    delay(1500);
    for (int i = 0; NBR_DISTANCEbr > i; i++)
    {
      executionStepParcours(tabDistancebr[i], tabAnglebr[i]);
    }
  }
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
>>>>>>> 7a4ddc1d12666e5726164297885eeeac711feb49

  case FALLOW_LINE2:

    break;

  case DETECT_COLOR:

    break;

  case PICKUP_BALL:

    break;

  case RED_PATH:

    break;

  case YELLOW_PATH:

    break;

  case BLUE_PATH:

    break;

  case FIND_WHITE_LINE:

    break;
  }*/
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
  Serial.println(voltageValue);
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
  { //les 2 roue tournes

    motorLeft = SPEED_MIN;

    //PID FOR ENCODER
    float error = abs(readEncoder0) - abs(readEncoder1);
    float p = error * KP_TURNING;

    sumError += error;
    float i = sumError * KI_FOWARD;

    float adjSpeed = p + i;
    motorRight = motorLeft + adjSpeed;

    if (angle > 0)
    { //test si l'angle est negative, tourne vers la droite
      motorLeft = -1 * motorLeft;
    }
    else if (angle < 0)
    { //test si l'angle est positive, tourne vers la gauche
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

    //PID FOR ENCODER
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
  float b = maxSpeed - (dDeceleration * deceleration); //pente de la fonction de decrementation

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
<<<<<<< HEAD
=======
void detectSound()
{
  float voltageValue = analogRead(ANALOG_BUZZER) * (5 / 1023.0);
  Serial.println(voltageValue);
  float target = 0.15;
  if (voltageValue >= target)
  {
    lightLED(1, 1, 1, 1);
    SOFT_TIMER_Disable(DETECT_SOUND_TIMER);
    SOFT_TIMER_Enable(DETECT_BOWLING_PIN_TIMER);
  }
}

void lightLED(bool red, bool green, bool blue, bool yellow)
{
  digitalWrite(DIGITAL_RED_LED, red);
  digitalWrite(DIGITAL_GREEN_LED, green);
  digitalWrite(DIGITAL_BLUE_LED, blue);
  digitalWrite(DIGITAL_YELLOW_LED, yellow);
}
>>>>>>> 7a4ddc1d12666e5726164297885eeeac711feb49

int detectColor()
{
    uint16_t clear, red, green, blue;
    char couleur[1];
    capteur.begin();
    delay(250); // takes 50ms to read
    capteur.getRawData(&red, &green, &blue, &clear);

<<<<<<< HEAD
    if (red > 500 && red < 600 && green > 525 && green < 625 && blue > 310 && blue < 410)
    {
      couleur[0] = 'j'; //test
      Serial.print(" \ncouleur\t ");
      Serial.print(couleur); //test
      return 1;
    }
    else if (red > 450 && red < 550 && green > 375 && green < 475 && blue > 320 && blue < 430)
    {
      couleur[0] = 'r'; //test
      Serial.print(" \ncouleur\t ");
      Serial.print(couleur); //test
      return 2;
    }
    else if (red > 350 && red < 450 && green > 460 && green < 560 && blue > 370 && blue < 470)
    {
      couleur[0] = 'b'; //test
      Serial.print(" \ncouleur\t ");
      Serial.print(couleur); //test
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
=======
void init_Detection5kHz(){
  PORTK |= (1<<7); //active PULL_DOWN A15
  PCICR |= (1<<2); //enable PCINT 23
  PCMSK2 |= (1<<7);
  sei();
}

/*ISR(PCINT2_vect){
  SOFT_TIMER_Disable(FOLLOW_LINE_TIMER);

} */
>>>>>>> 7a4ddc1d12666e5726164297885eeeac711feb49
