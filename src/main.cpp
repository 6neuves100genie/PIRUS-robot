
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
#include <Adafruit_GFX_Library/Adafruit_GFX.h>
#include <Adafruit_SSD1306_Library/Adafruit_SSD1306.h>
#include <unistd.h>
#include <SoftwareSerial.h>

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
#define PIN_ANALOG_CLE A2
// DIGITALS
#define PIN_BOUTON_BLEU 35
#define PIN_BOUTON_VERT 37
#define COLLECT_NUMBER 3 //Fingerprint sampling times, can be set to 1-3
#define IRQ 10           //IRQ pin

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
#define SPEED_MIN 0.22
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

//ALCOOTEST
int analogPin = 1;
int val = 0;
#define VAL_LIMITE 300

DFRobot_ID809 fingerprint;
Adafruit_SSD1306 display(-1);

SoftwareSerial Serialt(67, 53); //RX, TX
#define FPSerial Serialt

//pour les messages
char depot[] = "Deposer vos clefs sur le crochet.";
char identification[] = " appuyer votre doitg sur le TouchID jusqu'a ce qu'il clignote jaune 3 fois, repete cette etape jusqu'a ce qu'il clignotte vert";
char retrait[] = "Veuillez souffler sur le capteur pour recup vos clefs";
char debut[] = "Bouton vert pour dépot clefs       Bouton bleu pour retrait clefs";
char ID_retrait[] = "Appuyer le touch ID jusqu'a ce qu'il clignote bleu 3 fois";
char test_echoue[] = "Test echoue, veuillez degriser";
char test_reussite[] = "Bravo, vous pouvez conduire";

int xDebut, xID, xRetrait, xDepot, xID_R, xTest, xReussite;
int minDepot;
int minID;
int minDebut;
int minRetrait;
int minID_R;
int minTest;
int minReussite;

//compteur
int i = 0;
int y = 0;
int z = 0;
int r = 0;
int p = 0;
int scanDoigt = 0;

//gestion ID
#define VERT 1
#define ROUGE 2
#define BLEU 3
#define BLANC 4
#define NBR_ID 80
uint8_t tabID[NBR_ID][2]; // tableau, [ID], [couleur]
uint8_t cptID;
uint8_t numID; //pour le retrait

// NRF24L01
RF24 radio(48, 49); // CE, CSN
const byte address[6] = "00001";
char readDataCarousel[30];
char sendDataCarousel[] = "1";
void movingReverseRobot(uint16_t distance);
void NRF24L01_Init();
void NRF24L01_TransmitData(char *data, uint8_t size);
bool turnCarousel();
void suiveur_ligne();
void gauche();
void droite();
int detecterCle(int retrait); //0=depot   1=retrait
void turnedRobot180();
void manipul(int fonction);

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

typedef enum _etatClient
{
  DEPOT,
  RETRAIT,
  SAOUL,
} _etatClient;
_etatClient EtatClient;

// definition fonction
void executeStep();

// timer function
float voltageValue;

// motor function
void stopMotor();
void movingFowardRobot(uint16_t distance);
void turnedRobot(int angle);
float accelerationDecelerationPID(float pourcentageVitesse, uint8_t acceleration, uint8_t deceleration, float maxSpeed, float speed);

void servoMoteur(int angle);
int detectColor();

int readAlcohol();
void printAlcohol(int value);
void fingerprintMatching();
void fingerprintRegistration();
void infoReussite();
void infoEchec();
void infoID_R();
void infoID();
void infoDepot();
void infoRetrait();
void infoDebut();
void infoScanEnCours();
void delFinger();

/**
 * @brief 
 * 
 */
void setup()
{
  BoardInit();
  //Serial.begin(9600);
  NRF24L01_Init();

  Serial.println("test");

  //écran
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  display.setTextSize(3);
  display.setTextColor(WHITE);
  pinMode(PIN_BOUTON_BLEU, INPUT_PULLUP);
  pinMode(PIN_BOUTON_VERT, INPUT_PULLUP);
  display.setTextWrap(false);

  Serial.println("test");

  //affichage
  xDepot = display.width();
  xID = display.width();
  xDebut = display.width();
  xRetrait = display.width();
  minDepot = -12 * strlen(depot);
  minID = -12 * strlen(identification);
  minDebut = -12 * strlen(debut);
  minRetrait = -12 * strlen(retrait);
  minID_R = -12 * strlen(ID_retrait);
  minTest = -12 * strlen(test_echoue);
  minReussite = -12 * strlen(test_reussite);

  //setup touch ID
  cptID = 0;
  for (int i = 0; i < NBR_ID; i++)
  {
    tabID[i][0] = 0;
    tabID[i][1] = 0;
  }
  FPSerial.begin(115200);
  /*Take FPSerial as communication serial of fingerprint module*/
  fingerprint.begin(FPSerial);
  /*Wait for Serial to open*/
  while (!Serial)
    ;
  /*Test whether the device can properly communicate with mainboard
    Return true or false
    */

  Serial.println("test");

  while (fingerprint.isConnected() == false)
  {
    Serial.println("Communication with device failed, please check connection");
    //Get error code information
    //desc = fingerprint.getErrorDescription();
    Serial.println(fingerprint.getErrorDescription());
    delay(1000);
  }
  delFinger();

  Serial.println("test");

  readEncoder0 = 0;
  readEncoder1 = 0;

  Step = FIND_GOOD_KEY;
}

/**
 * @brief 
 * 
 */
void loop()
{
  executeStep();
}

/**
 * @brief 
 * 
 */
void executeStep()
{

  switch (Step)
  {
  case 0:
    display.clearDisplay();

    if (!digitalRead(PIN_BOUTON_BLEU) && digitalRead(PIN_BOUTON_VERT)) //retrait
    {
      Serial.println("RETRAIT");

      while (!scanDoigt)
      {
        if (digitalRead(IRQ))
        {
          infoScanEnCours();

          uint16_t i = 0;
          /*Capture fingerprint image, 5s idle timeout, if timeout=0,Disable  the collection timeout function
            Return 0 if succeed, otherwise return ERR_ID809
            */
          if ((fingerprint.collectionFingerprint(/*timeout=*/5)) != ERR_ID809)
          {
            /*Get the time finger pressed down*/
            /*Set fingerprint LED ring mode, color, and number of blinks 
              Can be set as follows:
              Parameter 1:<LEDMode>
              eBreathing   eFastBlink   eKeepsOn    eNormalClose
              eFadeIn      eFadeOut     eSlowBlink   
              Paramerer 2:<LEDColor>
              eLEDGreen  eLEDRed      eLEDYellow   eLEDBlue
              eLEDCyan   eLEDMagenta  eLEDWhite
              Parameter 3:<number of blinks> 0 represents blinking all the time
              This parameter will only be valid in mode eBreathing, eFastBlink, eSlowBlink
              */
            fingerprint.ctrlLED(/*LEDMode = */ fingerprint.eFastBlink, /*LEDColor = */ fingerprint.eLEDBlue, /*blinkCount = */ 3); //blue LED blinks quickly 3 times, means it's in fingerprint comparison mode now
            /*Wait for finger to relase */
            while (fingerprint.detectFinger())
            {
              delay(50);
              i++;
              if (i == 15)
              { //Yellow LED blinks quickly 3 times, means it's in fingerprint regisrtation mode now
                /*Set fingerprint LED ring to always ON in yellow*/
                fingerprint.ctrlLED(/*LEDMode = */ fingerprint.eFastBlink, /*LEDColor = */ fingerprint.eLEDYellow, /*blinkCount = */ 3);
              }
              else if (i == 30)
              { //Red LED blinks quickly 3 times, means it's in fingerprint deletion mode now
                /*Set fingerprint LED ring to always ON in red*/
                fingerprint.ctrlLED(/*LEDMode = */ fingerprint.eFastBlink, /*LEDColor = */ fingerprint.eLEDRed, /*blinkCount = */ 3);
              }
            }
          }
          if (i == 0)
          {
            /*Fingerprint capturing failed*/
          }
          else if (i > 0 && i < 15)
          {
            Serial.println("Enter fingerprint comparison mode");
            /*Compare fingerprints*/
            fingerprintMatching();
          }
        }
        else
        {
          infoID_R();
          delay(50);
        }
      }
      scanDoigt = 0;
      Serial.println("************************************************************************************");

      while (p <= 100)
      {
        display.clearDisplay();

        val = readAlcohol();
        printAlcohol(val);
        delay(50);
        Serial.println(val);
        p++;
      }

      if (VAL_LIMITE <= val)
      { //saoul
        infoEchec();
        EtatClient = SAOUL;
        Serial.println("/*****SAOUL***********/");
      }
      else
      { //pas saoul
        infoReussite();
        EtatClient = RETRAIT;
        Step = GO_TO_CAROUSEL;
        Serial.println("/*****PAS SAOUL***********/");
      }
    }
    else if (!digitalRead(PIN_BOUTON_VERT) && digitalRead(PIN_BOUTON_BLEU)) // depot clée
    {
      Serial.println("depot clee");

      while (!scanDoigt)
      {
        if (digitalRead(IRQ))
        {
          infoScanEnCours();

          uint16_t i = 0;
          /*Capture fingerprint image, 5s idle timeout, if timeout=0,Disable  the collection timeout function
            Return 0 if succeed, otherwise return ERR_ID809
            */
          if ((fingerprint.collectionFingerprint(/*timeout=*/5)) != ERR_ID809)
          {
            /*Get the time finger pressed down*/
            /*Set fingerprint LED ring mode, color, and number of blinks 
              Can be set as follows:
              Parameter 1:<LEDMode>
              eBreathing   eFastBlink   eKeepsOn    eNormalClose
              eFadeIn      eFadeOut     eSlowBlink   
              Paramerer 2:<LEDColor>
              eLEDGreen  eLEDRed      eLEDYellow   eLEDBlue
              eLEDCyan   eLEDMagenta  eLEDWhite
              Parameter 3:<number of blinks> 0 represents blinking all the time
              This parameter will only be valid in mode eBreathing, eFastBlink, eSlowBlink
              */
            fingerprint.ctrlLED(/*LEDMode = */ fingerprint.eFastBlink, /*LEDColor = */ fingerprint.eLEDBlue, /*blinkCount = */ 3); //blue LED blinks quickly 3 times, means it's in fingerprint comparison mode now
            /*Wait for finger to relase */
            while (fingerprint.detectFinger())
            {
              delay(50);
              i++;
              if (i == 15)
              { //Yellow LED blinks quickly 3 times, means it's in fingerprint regisrtation mode now
                /*Set fingerprint LED ring to always ON in yellow*/
                fingerprint.ctrlLED(/*LEDMode = */ fingerprint.eFastBlink, /*LEDColor = */ fingerprint.eLEDYellow, /*blinkCount = */ 3);
              }
              else if (i == 30)
              { //Red LED blinks quickly 3 times, means it's in fingerprint deletion mode now
                /*Set fingerprint LED ring to always ON in red*/
                fingerprint.ctrlLED(/*LEDMode = */ fingerprint.eFastBlink, /*LEDColor = */ fingerprint.eLEDRed, /*blinkCount = */ 3);
              }
            }
          }
          if (i == 0)
          {
            /*Fingerprint capturing failed*/
          }
          else if (i >= 15 && i < 30)
          {
            Serial.println("Enter fingerprint registration mode");
            /*Registrate fingerprint*/
            fingerprintRegistration();
            Serial.println(tabID[cptID - 1][0]);
            scanDoigt = 1;
          }
        }
        else
        {
          infoID();
          delay(50);
        }
      }
      scanDoigt = 0;
      detecterCle(0);
      EtatClient = DEPOT;
      Step = GO_TO_CAROUSEL;
      Serial.println(Step);
    }
    else
      infoDebut();
    break;

  case 1:
    Serial.println("************************************************************************************");
    suiveur_ligne();
    Step = FIND_GOOD_KEY;
    Serial.println("************************************************************************************");
    break;

  case 2:

    Serial.println("************************************************************************************");
    //int Couleur_actuel = detectColor();
    
    if (EtatClient == RETRAIT)
    {
      turnCarousel();
      Step = TAKE_KEY;
    }
    if (EtatClient == DEPOT)
    {
      Step = DROP_KEY;
    }
    break;

  case 3:
    turnedRobot180();
    delay(50);
    turnedRobot180();
    delay(50);
    manipul(1);
    Step = GO_BACK;
    break;

  case 4:
    Serial.println("************************************************************************************");
    turnedRobot180();
    delay(50);
    turnedRobot180();
    delay(50);
    manipul(2);
    Step = GO_BACK;
    break;

  case 5:

    Step = GO_BACK;
    break;

  case 6:
    suiveur_ligne();
    delay(50);
    turnedRobot180();
    delay(50);
    turnedRobot180();
    Step = IDENTIFICATION;
    break;
  }
}

/**
 * @brief 
 * 
 */
void stopMotor()
{
  MOTOR_SetSpeed(RIGHT_WHEEL, 0);
  MOTOR_SetSpeed(LEFT_WHEEL, 0);
}

/**
 * @brief 
 * 
 * @param angle 
 */
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

/**
 * @brief 
 * 
 * @param distance 
 */
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

    readEncoder0 = ENCODER_Read(LEFT_WHEEL) * -1;
    readEncoder1 = ENCODER_Read(RIGHT_WHEEL) * -1;

    //float pourcentageVitesse = float(readEncoder0/distanceEncodeur) * 100;
    //PID FOR ENCODER
    float error = readEncoder0 - readEncoder1;
    float p = error * KP_FOWARD;

    sumError += error;
    float i = sumError * KI_FOWARD;

    float adjSpeed = p + i;
    motorRight = motorLeft + adjSpeed;

    MOTOR_SetSpeed(LEFT_WHEEL, -0.2);
    MOTOR_SetSpeed(RIGHT_WHEEL, -0.2);
  }
}

void movingReverseRobot(uint16_t distance)
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

    //float pourcentageVitesse = float(readEncoder0/distanceEncodeur) * 100;
    //PID FOR ENCODER
    float error = readEncoder0 - readEncoder1;
    float p = error * KP_FOWARD;

    sumError += error;
    float i = sumError * KI_FOWARD;

    float adjSpeed = p + i;
    motorRight = motorLeft + adjSpeed;

    MOTOR_SetSpeed(LEFT_WHEEL, 0.2);
    MOTOR_SetSpeed(RIGHT_WHEEL, 0.2);
  }
}

/**
 * @brief 
 * 
 * @param pourcentageVitesse 
 * @param acceleration 
 * @param deceleration 
 * @param maxSpeed 
 * @param speed 
 * @return float 
 */
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

/**
 * @brief 
 * 
 * @return int 
 */
int detectColor()
{
  uint16_t clear, red, green, blue;
  char couleur[1];
  delay(200); // takes 50ms to read
  capteur.getRawData(&red, &green, &blue, &clear);

  float x = (-0.14282 * red) + (1.54924 * green) + (-0.95641 * blue);
  float y = (-0.32466 * red) + (1.57837 * green) + (-0.73191 * blue);
  float z = (-0.68202 * red) + (0.77073 * green) + (-0.56332 * blue);
  float xx = x / (x + y + z);
  float yy = y / (x + y + z);

  if (xx > -7 && xx < 0 && yy > -10 && yy < 0)
  {
    couleur[0] = 'r';
    Serial.print(" \ncouleur\t ");
    Serial.print(couleur);
    return 1;
  }
  else if (xx > 0.5 && xx < 0.6 && yy > 0.75 && yy < 0.85)
  {
    couleur[0] = 'b';
    Serial.print(" \ncouleur\t ");
    Serial.print(couleur);
    return 2;
  }
  else if (xx > 0.4 && xx < 0.55 && yy > 0.45 && yy < 0.6)
  {
    couleur[0] = 'g';
    Serial.print(" \ncouleur\t ");
    Serial.print(couleur);
    return 3;
  }
  else if (xx > 0.55 && xx < 0.70 && yy > 0.65 && yy < 0.75)
  {
    couleur[0] = 'w';
    Serial.print(" \ncouleur\t ");
    Serial.print(couleur);
    return 3;
  }

  Serial.print("C:");
  Serial.print(clear);
  Serial.print("\tR:");
  Serial.print(red);
  Serial.print("\tG:");
  Serial.print(green);
  Serial.print("\tB:");
  Serial.print(blue);
  Serial.print("\t X:");
  Serial.print(xx);
  Serial.print("\tY:\n");
  Serial.print(yy);
  return 0;
}

/**
 * @brief 
 * 
 * @param angle 
 */
void servoMoteur(int angle)
{
  if (angle > angleMoteur)
  {
    for (int i = angleMoteur; i < angle; i += 1)
    {
      SERVO_SetAngle(0, i);
      delay(15);
    }
  }
  else
  {
    for (int i = angleMoteur; i > angle; i -= 1)
    {
      SERVO_SetAngle(0, i);
      delay(15);
    }
  }
  angleMoteur = angle;
}

/**
 * @brief 
 * 
 */
void NRF24L01_Init()
{
  radio.begin();
  // receiver
  radio.openReadingPipe(0, address);

  // transmitter
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
}

/**
 * @brief 
 * 
 * @param data 
 * @param size 
 */
void NRF24L01_TransmitData(char *data, uint8_t size)
{
  radio.stopListening();

  radio.write(data, size);
  //Serial.println(data);

  radio.startListening();

  delay(250);
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool turnCarousel()
{
  NRF24L01_TransmitData(sendDataCarousel, sizeof(sendDataCarousel));

  int cpt = 0;
  while (!radio.available())
  { // attent le retour de commande carousel
    delay(1);
    cpt++;
    if (cpt >= 3000) //evite de rester dans le while
      return 0;
  }
  Serial.println("OUI");
  radio.read(&readDataCarousel, sizeof(readDataCarousel));
  Serial.println(readDataCarousel);

  if (readDataCarousel[0] == '1')
    return 1;
  else
    return 0;
}

void suiveur_ligne()
{
  Serial.println(voltageValue);
  MOTOR_SetSpeed(LEFT_WHEEL, 0);
  MOTOR_SetSpeed(RIGHT_WHEEL, 0);
  voltageValue = (analogRead(ANALOG_LINE_FOLLOWER)) * (5 / 1023.0);
  Serial.println(voltageValue);
  while (voltageValue > 0 && voltageValue < 4.5)
  {
    MOTOR_SetSpeed(LEFT_WHEEL, -0.2);
    MOTOR_SetSpeed(RIGHT_WHEEL, -0.2);
    voltageValue = (analogRead(ANALOG_LINE_FOLLOWER)) * (5 / 1023.0);
    Serial.println(voltageValue);
    if (voltageValue > 1.4 && voltageValue < 1.7)
    {
      gauche();
    }
    if (voltageValue > 0.6 && voltageValue < 1)
    {
      droite();
    }
    delay(100);
    voltageValue = (analogRead(ANALOG_LINE_FOLLOWER)) * (5 / 1023.0);
  }
  MOTOR_SetSpeed(LEFT_WHEEL, 0);
  MOTOR_SetSpeed(RIGHT_WHEEL, 0);
  Serial.println("fin");
  delay(500);
}

void gauche()
{
  Serial.println("gauche");
  for (voltageValue = (analogRead(ANALOG_LINE_FOLLOWER)) * (5 / 1023.0); voltageValue < 2.7 || voltageValue > 2.9;)
  {
    MOTOR_SetSpeed(LEFT_WHEEL, -0.1);
    MOTOR_SetSpeed(RIGHT_WHEEL, -0.15);
    voltageValue = (analogRead(ANALOG_LINE_FOLLOWER)) * (5 / 1023.0);
    Serial.println(voltageValue);
  }
  for (int i = 0; i < 1; i++)
  {
    MOTOR_SetSpeed(LEFT_WHEEL, -0.15);
    MOTOR_SetSpeed(RIGHT_WHEEL, -0.1);
    delay(35);
  }
}

void droite()
{
  Serial.println("droite");
  for (voltageValue = (analogRead(ANALOG_LINE_FOLLOWER)) * (5 / 1023.0); voltageValue < 2.7 || voltageValue > 2.9;)
  {
    MOTOR_SetSpeed(LEFT_WHEEL, -0.15);
    MOTOR_SetSpeed(RIGHT_WHEEL, -0.1);
    voltageValue = (analogRead(ANALOG_LINE_FOLLOWER)) * (5 / 1023.0);
    Serial.println(voltageValue);
  }
  for (int i = 0; i < 1; i++)
  {
    MOTOR_SetSpeed(LEFT_WHEEL, -0.1);
    MOTOR_SetSpeed(RIGHT_WHEEL, -0.15);
    delay(35);
  }
}

/**
 * @brief 
 * 
 * @param retrait 1=retrait 0=depot
 * @return int 
 */
int detecterCle(int retrait)
{
  //servoMoteur(90);
  Serial.println("dc");
  float valeurBase = analogRead(PIN_ANALOG_CLE) * (5 / 1023.0);
  Serial.println(valeurBase);
  while (retrait == 0)
  {
    delay(50);
    float ValeurActuel = analogRead(PIN_ANALOG_CLE) * (5 / 1023.0);
    Serial.println(ValeurActuel);
    if (ValeurActuel > valeurBase + 0.02)
    {
      delay(1000);
      //retrait == 2;
      return 1;
    }
  }
  while (retrait == 1)
  {
    delay(50);
    float ValeurActuel = analogRead(PIN_ANALOG_CLE) * (5 / 1023.0);
    Serial.println(ValeurActuel);
    if (ValeurActuel < valeurBase - 0.02)
    {
      delay(1000);
      return 1;
    }
  }
  return 0;
}

void turnedRobot180()
{
  MOTOR_SetSpeed(LEFT_WHEEL, -0.2);
  MOTOR_SetSpeed(RIGHT_WHEEL, 0.2);
  delay(1000);
  for (voltageValue = (analogRead(ANALOG_LINE_FOLLOWER)) * (5 / 1023.0); voltageValue < 2.7 || voltageValue > 2.9;)
  {
    MOTOR_SetSpeed(LEFT_WHEEL, -0.2);
    MOTOR_SetSpeed(RIGHT_WHEEL, 0.2);
    voltageValue = (analogRead(ANALOG_LINE_FOLLOWER)) * (5 / 1023.0);
    Serial.println(voltageValue);
  }
  MOTOR_SetSpeed(LEFT_WHEEL, 0);
  MOTOR_SetSpeed(RIGHT_WHEEL, 0);
}

void manipul(int fonction)
{

  if (fonction == 1)
  {
    servoMoteur(90);
    delay(1000);
    movingReverseRobot(9.5);
    delay(50);
    turnedRobot(-5);
    delay(1000);
    servoMoteur(70);
    delay(1000);
    turnedRobot(5);
    delay(1000);
  }
  if (fonction == 2)
  {
    servoMoteur(70);
    delay(1000);
    movingReverseRobot(9.5);
    delay(50);
    turnedRobot(-5);
    delay(1000);
    servoMoteur(90);
    delay(1000);
    turnedRobot(5);
    delay(1000);
  }
  movingFowardRobot(2);
  servoMoteur(90);
}

void infoDebut()
{
  display.clearDisplay();
  display.setCursor(0, 7);
  display.setTextSize(1);
  display.print("Debut");
  display.setTextSize(2);
  display.setCursor(xDebut, 15);
  display.print(debut);
  display.display();
  xDebut = xDebut - 4;
  if (xDebut < minDebut)
    xDebut = display.width();
}

void infoRetrait()
{
  display.clearDisplay();
  display.setCursor(0, 7);
  display.setTextSize(1);
  display.print("Retrait");
  display.setTextSize(2);
  display.setCursor(xRetrait, 15);
  display.print(retrait);
  display.display();
  xRetrait = xRetrait - 4;
  if (xRetrait < minRetrait)
    xRetrait = display.width();
}

void infoDepot()
{
  display.clearDisplay();
  display.setCursor(0, 7);
  display.setTextSize(1);
  display.print("Depot");
  display.setTextSize(2);
  display.setCursor(xDepot, 15);
  display.print(depot);
  display.display();
  xDepot = xDepot - 4;
  if (xDepot < minDepot)
    xDepot = display.width();
}

void infoID()
{
  display.clearDisplay();
  display.setCursor(0, 7);
  display.setTextSize(1);
  display.print("ID");
  display.setTextSize(2);
  display.setCursor(xID, 15);
  display.print(identification);
  display.display();
  xID = xID - 8;
  if (xID < minID)
    xID = display.width();
}

void infoID_R()
{
  display.clearDisplay();
  display.setCursor(0, 7);
  display.setTextSize(1);
  display.print("ID");
  display.setTextSize(2);
  display.setCursor(xID_R, 15);
  display.print(ID_retrait);
  display.display();
  xID_R = xID_R - 7;
  if (xID_R < minID_R)
    xID_R = display.width();
}

void infoEchec()
{
  display.clearDisplay();
  display.setCursor(0, 7);
  display.setTextSize(1);
  display.print("Echec");
  display.setTextSize(2);
  display.setCursor(xTest, 15);
  display.print(test_echoue);
  display.display();
  xTest = xTest - 7;
  if (xTest < minTest)
    xTest = display.width();
}

void infoReussite()
{
  display.clearDisplay();
  display.setCursor(0, 7);
  display.setTextSize(1);
  display.print("Reussite");
  display.setTextSize(2);
  display.setCursor(xReussite, 15);
  display.print(test_reussite);
  display.display();
  xReussite = xReussite - 7;
  if (xReussite < minReussite)
    xReussite = display.width();
}

void infoScanEnCours()
{
  display.clearDisplay();
  display.setCursor(20, 10);
  display.setTextSize(2);
  display.print("SCAN ID");
  display.display();
}

void fingerprintRegistration()
{
  uint8_t ID, i;
  /*Compare the captured fingerprint with all fingerprints in the fingerprint library
    Return fingerprint ID number(1-80) if succeed, return 0 when failed
    Function: clear the last captured fingerprint image
   */
  fingerprint.search(); //Can add "if else" statement to judge whether the fingerprint has been registered.
  /*Get a unregistered ID for saving fingerprint 
    Return ID number when succeed 
    Return ERR_ID809 if failed
   */
  if ((ID = fingerprint.getEmptyID()) == ERR_ID809)
  {
    while (1)
    {
      /*Get error code imformation*/
      //desc = fingerprint.getErrorDescription();
      //Serial.println(desc);
      delay(1000);
    }
  }
  Serial.print("Unregistered ID,ID=");
  Serial.println(ID);
  i = 0; //Clear sampling times
  /*Fingerprint Sampling 3 times */
  while (i < COLLECT_NUMBER)
  {
    /*Set fingerprint LED ring to breathing lighting in blue*/
    fingerprint.ctrlLED(/*LEDMode = */ fingerprint.eBreathing, /*LEDColor = */ fingerprint.eLEDBlue, /*blinkCount = */ 0);
    Serial.print("The fingerprint sampling of the");
    Serial.print(i + 1);
    Serial.println("(th) time is being taken");
    Serial.println("Please press down your finger");
    /*Capture fingerprint image, 10s idle timeout 
      If succeed return 0, otherwise return ERR_ID809
     */
    if ((fingerprint.collectionFingerprint(/*timeout = */ 10)) != ERR_ID809)
    {
      /*Set fingerprint LED ring to quick blink in yellow 3 times*/
      fingerprint.ctrlLED(/*LEDMode = */ fingerprint.eFastBlink, /*LEDColor = */ fingerprint.eLEDYellow, /*blinkCount = */ 3);
      Serial.println("Capturing succeeds");
      i++; //Sampling times +1
    }
    else
    {
      Serial.println("Capturing fails");
      /*Get error code information*/
      //desc = fingerprint.getErrorDescription();
      //Serial.println(desc);
    }
    Serial.println("Please release your finger");
    /*Wait for finger to release
      Return 1 when finger is detected, otherwise return 0 
     */
    while (fingerprint.detectFinger())
      ;
  }

  /*Save fingerprint information into an unregistered ID*/
  if (fingerprint.storeFingerprint(/*Empty ID = */ ID) != ERR_ID809)
  {

    tabID[cptID++][0] = ID;

    Serial.print("Saving succeed，ID=");
    Serial.println(ID);
    /*Set fingerprint LED ring to always ON in green*/
    fingerprint.ctrlLED(/*LEDMode = */ fingerprint.eKeepsOn, /*LEDColor = */ fingerprint.eLEDGreen, /*blinkCount = */ 0);
    delay(1000);
    /*Turn off fingerprint LED ring */
    fingerprint.ctrlLED(/*LEDMode = */ fingerprint.eNormalClose, /*LEDColor = */ fingerprint.eLEDBlue, /*blinkCount = */ 0);
  }
  else
  {
    Serial.println("Saving failed");
    /*Get error code information*/
    //desc = fingerprint.getErrorDescription();
    //Serial.println(desc);
  }
  Serial.println("-----------------------------");
}

void fingerprintMatching()
{
  /*Compare the captured fingerprint with all fingerprints in the fingerprint library
    Return fingerprint ID number(1-80) if succeed, return 0 when failed
   */
  uint8_t ret = fingerprint.search();
  if (ret != 0)
  {
    /*Set fingerprint LED ring to always ON in green*/
    fingerprint.ctrlLED(/*LEDMode = */ fingerprint.eKeepsOn, /*LEDColor = */ fingerprint.eLEDGreen, /*blinkCount = */ 0);
    Serial.print("Successfully matched,ID=");
    Serial.println(ret);
    numID = ret;
    scanDoigt = 1;
  }
  else
  {
    /*Set fingerprint LED Ring to always ON in red*/
    fingerprint.ctrlLED(/*LEDMode = */ fingerprint.eKeepsOn, /*LEDColor = */ fingerprint.eLEDRed, /*blinkCount = */ 0);
    Serial.println("Matching failed");
  }
  delay(1000);
  /*Turn off fingerprint LED Ring*/
  fingerprint.ctrlLED(/*LEDMode = */ fingerprint.eNormalClose, /*LEDColor = */ fingerprint.eLEDBlue, /*blinkCount = */ 0);
  Serial.println("-----------------------------");
}

void printAlcohol(int value)
{
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(45, 10);
  display.println(val);
  display.display();
}

int readAlcohol()
{
  int val = 0;
  int val1;
  int val2;
  int val3;
  int val4;
  int val5;
  int val6;
  int val7;

  int val8;
  int val9;
  int val10;
  int val11;
  int val12;
  int val13;
  int val14;
  int val15;

  display.clearDisplay();
  val1 = analogRead(analogPin);
  delay(20);
  val2 = analogRead(analogPin);
  delay(20);
  val3 = analogRead(analogPin);
  delay(20);
  val4 = analogRead(analogPin);
  delay(20);
  val5 = analogRead(analogPin);
  delay(20);
  val6 = analogRead(analogPin);
  delay(20);
  val7 = analogRead(analogPin);
  delay(20);
  val8 = analogRead(analogPin);
  delay(20);
  val9 = analogRead(analogPin);
  delay(20);
  val10 = analogRead(analogPin);
  delay(20);
  val11 = analogRead(analogPin);
  delay(20);
  val13 = analogRead(analogPin);
  delay(20);
  val14 = analogRead(analogPin);
  delay(20);
  val5 = analogRead(analogPin);

  val = (val1 + val2 + val3 + val4 + val5 + val6 + val7 + val8 + val9 + val10 + val11 + val12 + val13 + val14 + val15) / 15;
  return val;
}

void delFinger()
{

  fingerprint.delFingerprint(1);
  fingerprint.delFingerprint(2);
  fingerprint.delFingerprint(3);
  fingerprint.delFingerprint(5);
  fingerprint.delFingerprint(6);
  fingerprint.delFingerprint(7);
  fingerprint.delFingerprint(8);
  fingerprint.delFingerprint(9);
  fingerprint.delFingerprint(10);
  fingerprint.delFingerprint(11);
  fingerprint.delFingerprint(12);
  fingerprint.delFingerprint(13);
  fingerprint.delFingerprint(14);
  fingerprint.delFingerprint(15);
  fingerprint.delFingerprint(16);
  fingerprint.delFingerprint(17);
  fingerprint.delFingerprint(18);
  fingerprint.delFingerprint(19);
  fingerprint.delFingerprint(20);
}