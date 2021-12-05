
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
#define PIN_BOUTON_VERT 36
#define COLLECT_NUMBER 3 //Fingerprint sampling times, can be set to 1-3
#define IRQ 6            //IRQ pin

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

//ALCOOTEST
int analogPin = 1;
int val = 0;

DFRobot_ID809 fingerprint;
Adafruit_SSD1306 display(-1);

SoftwareSerial Serialt(53, 3); //RX, TX
#define FPSerial Serialt

//pour les messages
char depot[] = "Deposer vos clefs sur le crochet.";
char identification[] = " appuyer votre doitg sur le TouchID jusqu'a ce qu'il clignote jaune 3 fois, répété cette étape jusqu'a ce qu'il clignotte vert";
char retrait[] = "Veuillez souffler sur le capteur pour recup vos clefs";
char debut[] = "Bouton vert pour dépot clefs       Bouton bleu pour retrait clefs";
char ID_retrait[] = "Appuyer le touch ID jusqu'a ce qu'il clignote bleu 3 fois";
char test_echoue[] = "Test echoue, veuillez dégriser";
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
int w = 0;

//gestion ID
#define VERT 1
#define ROUGE 2
#define BLEU 3
#define BLANC 4
#define NBR_ID 80
uint8_t tabID[NBR_ID][2]; // tableau, [ID], [couleur]
uint8_t cptID;

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
int detecterCle(int retrait);     //0=depot   1=retrait
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

/**
 * @brief 
 * 
 */
void setup()
{
  BoardInit();
  NRF24L01_Init();

  //écran
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  display.setTextSize(3);
  display.setTextColor(WHITE);
  pinMode(PIN_BOUTON_BLEU, INPUT_PULLUP);
  pinMode(PIN_BOUTON_VERT, INPUT_PULLUP);
  display.setTextWrap(false);

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
  while (fingerprint.isConnected() == false)
  {
    Serial.println("Communication with device failed, please check connection");
    /*Get error code information*/
    //desc = fingerprint.getErrorDescription();
    //Serial.println(desc);
    delay(1000);
  }

  readEncoder0 = 0;
  readEncoder1 = 0;

  Step = IDENTIFICATION;
}

/**
 * @brief 
 * 
 */
void loop()
{

  bool value = turnCarousel();
  Serial.println(value);

  if (value)
    Serial.println("fonctionne!!!");
  else
    Serial.println("fonctionne pas...");

  delay(1000);

  // executeStep();
}

/**
 * @brief 
 * 
 */
void executeStep()
{

  switch (Step)
  {
  case IDENTIFICATION:
    
    Step = GO_TO_CAROUSEL;
    break;

  case GO_TO_CAROUSEL:
    //suiveur_ligne();
   /* delay(2000);
    bool val = turnCarousel();
    Serial.println(val);*/
    //Step = FIND_GOOD_KEY;
    detectColor();
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


  while(((distanceEncodeur ) > readEncoder0) && ((distanceEncodeur) > readEncoder1)){

    readEncoder0 = ENCODER_Read(LEFT_WHEEL)*-1;  
    readEncoder1 = ENCODER_Read(RIGHT_WHEEL)*-1;

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

void movingReverseRobot(uint16_t distance){
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


  while(((distanceEncodeur ) > readEncoder0) && ((distanceEncodeur) > readEncoder1)){

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
    delay(200);  // takes 50ms to read
    capteur.getRawData(&red, &green, &blue, &clear);


    float x =(-0.14282*red)+(1.54924*green)+(-0.95641*blue);
    float y =(-0.32466*red)+(1.57837*green)+(-0.73191*blue);
    float z =(-0.68202*red)+(0.77073*green)+(-0.56332*blue);
    float xx=x/(x+y+z);
    float yy=y/(x+y+z);

    if (xx>-7 && xx<0 && yy>-10 && yy<0)
    {
        couleur[0]='r';
        Serial.print(" \ncouleur\t "); Serial.print(couleur);
        return 1;
    }
        else if (xx>0.5 && xx<0.6 && yy>0.75 && yy<0.85)
    {
        couleur[0]='b';
        Serial.print(" \ncouleur\t "); Serial.print(couleur);
        return 2;
    }
    else  if (xx>0.4 && xx<0.55 && yy>0.45 && yy<0.6)
    {
        couleur[0]='g';
        Serial.print(" \ncouleur\t "); Serial.print(couleur);
        return 3;
    }
    else  if (xx>0.55 && xx<0.70 && yy>0.65 && yy<0.75)
    {
        couleur[0]='w';
        Serial.print(" \ncouleur\t "); Serial.print(couleur);
        return 3;
    }

    Serial.print("C:"); Serial.print(clear);
    Serial.print("\tR:"); Serial.print(red);
    Serial.print("\tG:"); Serial.print(green);
    Serial.print("\tB:"); Serial.print(blue);
    Serial.print("\t X:"); Serial.print(xx);
    Serial.print("\tY:\n"); Serial.print(yy);
    return 0;
}

void servoMoteur(int angle){
    if (angle>angleMoteur)
    {
        for (int i=angleMoteur;i<angle;i+=1){
        SERVO_SetAngle(0,i);
        delay(15);
  }
}
    else{
      for (int i=angleMoteur;i>angle;i-=1){
      SERVO_SetAngle(0,i);
      delay(15);
      }
}
  angleMoteur=angle;
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

void suiveur_ligne(){
  
Serial.println(voltageValue);
MOTOR_SetSpeed(LEFT_WHEEL,0);
MOTOR_SetSpeed(RIGHT_WHEEL,0);
voltageValue = (analogRead(ANALOG_LINE_FOLLOWER)) * (5 / 1023.0);
Serial.println(voltageValue);
while (voltageValue>0&&voltageValue<4.5)
    {
        MOTOR_SetSpeed(LEFT_WHEEL,-0.2);
        MOTOR_SetSpeed(RIGHT_WHEEL,-0.2);
        voltageValue = (analogRead(ANALOG_LINE_FOLLOWER)) * (5 / 1023.0);
        Serial.println(voltageValue);
        if (voltageValue>1.4&&voltageValue<1.7)
            {
            gauche();
            }
        if (voltageValue>0.6&&voltageValue<1)
            {
            droite();
            }
            delay(100);
            voltageValue = (analogRead(ANALOG_LINE_FOLLOWER)) * (5 / 1023.0);
    }
MOTOR_SetSpeed(LEFT_WHEEL,0);
MOTOR_SetSpeed(RIGHT_WHEEL,0);
Serial.println("fin");
delay(500);

}

void gauche(){
    Serial.println("gauche");
    for(voltageValue = (analogRead(ANALOG_LINE_FOLLOWER)) * (5 / 1023.0);voltageValue<2.7 || voltageValue>2.9;)
    {
    MOTOR_SetSpeed(LEFT_WHEEL,-0.1);
    MOTOR_SetSpeed(RIGHT_WHEEL,-0.15);
    voltageValue = (analogRead(ANALOG_LINE_FOLLOWER)) * (5 / 1023.0);
    Serial.println(voltageValue);
    }
    for(int i=0;i<1;i++){
    MOTOR_SetSpeed(LEFT_WHEEL,-0.15);
    MOTOR_SetSpeed(RIGHT_WHEEL,-0.1);
    delay(35);
    }
}

void droite(){
     Serial.println("droite");
    for(voltageValue = (analogRead(ANALOG_LINE_FOLLOWER)) * (5 / 1023.0);voltageValue<2.7 ||voltageValue>2.9;){
    MOTOR_SetSpeed(LEFT_WHEEL,-0.15);
    MOTOR_SetSpeed(RIGHT_WHEEL,-0.1);
    voltageValue = (analogRead(ANALOG_LINE_FOLLOWER)) * (5 / 1023.0);
    Serial.println(voltageValue);}
    for(int i=0;i<1;i++){
    MOTOR_SetSpeed(LEFT_WHEEL,-0.1);
    MOTOR_SetSpeed(RIGHT_WHEEL,-0.15);
    delay(35);
    }
}

int detecterCle(int retrait)
{
  servoMoteur(90);
  Serial.println("dc");
  float valeurBase = analogRead(PIN_ANALOG_CLE)* (5 / 1023.0);
  Serial.println(valeurBase);
  while (retrait==0)
  {
    delay(50);
    float  ValeurActuel = analogRead(PIN_ANALOG_CLE)* (5 / 1023.0);
    Serial.println(ValeurActuel);
    if (ValeurActuel > valeurBase+0.02)
    {
      delay(1000);
      return 1;
    }
  }
    while (retrait==1)
  {
    delay(50);
    float  ValeurActuel = analogRead(PIN_ANALOG_CLE)* (5 / 1023.0);
    Serial.println(ValeurActuel);
    if (ValeurActuel < valeurBase-0.02)
    {
      delay(1000);
      return 1;
    }
  }
  return 0;
}

void turnedRobot180(){
  MOTOR_SetSpeed(LEFT_WHEEL,-0.2);
  MOTOR_SetSpeed(RIGHT_WHEEL,0.2);
  delay(1000);
for(voltageValue = (analogRead(ANALOG_LINE_FOLLOWER)) * (5 / 1023.0);voltageValue<2.7 ||voltageValue>2.9;){
    MOTOR_SetSpeed(LEFT_WHEEL,-0.2);
    MOTOR_SetSpeed(RIGHT_WHEEL,0.2);
    voltageValue = (analogRead(ANALOG_LINE_FOLLOWER)) * (5 / 1023.0);
    Serial.println(voltageValue);}
    MOTOR_SetSpeed(LEFT_WHEEL,0);
    MOTOR_SetSpeed(RIGHT_WHEEL,0);
}

void manipul(int fonction){
    
    if (fonction==1){
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
     if (fonction==2){
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
