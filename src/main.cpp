
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


#define FOLLOW_LINE_TIMER 1
#define DETECT_BOWLING_PIN_TIMER 2
#define DETECT_SOUND_TIMER 3 

// OBP-704 possible values for suiveur de ligne
#define LINE_LEFT  0.25             // 1 0 0 0
#define LINE_LEANING_LEFT 0.95      // 1 1 0 0
#define LINE_SLIGHTLY_LEFT 0.78     // 0 1 0 0
#define LINE_CENTER 2.16            // 0 1 1 0
#define LINE_SLIGHTLY_RIGHT 1.45    // 0 0 1 0
#define LINE_LEANING_RIGHT 4.19     // 0 0 1 1
#define LINE_RIGHT 2.81             // 0 0 0 1
#define LINE_LOST 0.07              // 0 0 0 0


//WHEEL
#define ENCODER_STEP 3200
#define WHEEL_DIAMETER 7.62 //cm
#define WHEEL_CIRCUMFERENCE (WHEEL_DIAMETER*PI)
#define RESOLUTION_ENCODER (WHEEL_CIRCUMFERENCE/ENCODER_STEP)
#define ROBOT_CIRCUMFERENCE  (19.05*PI)
#define RIGHT_WHEEL 1
#define LEFT_WHEEL 0

//FOWARD
#define SPEED_MAX_HIGH_DISTANCE 0.9
#define SPEED_MAX_LOW_DISTANCE 0.6
#define SPEED_MIN 0.2
#define ACCELERATION_HIGH_DISTANCE 20 //accelere jusqu'a 10% de la distance
#define DECCELERATION_HIGH_DISTANCE 75 //deccelere a partir de 90% de la distance
#define ACCELERATION_LOW_DISTANCE 20 //accelere jusqu'a 10% de la distance
#define DECCELERATION_LOW_DISTANCE 75 //deccelere a partir de 90% de la distance

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

//TIMER0
volatile uint8_t compt = 0;
volatile bool encoderEqual = false;

//ETAPE PARCOURS
#define NBR_DISTANCE   1
#define NBR_DISTANCEr  1
#define NBR_DISTANCErr 2
#define NBR_DISTANCEj  3
#define NBR_DISTANCEjr 4
#define NBR_DISTANCEb  3
#define NBR_DISTANCEbr 4
#define NBR_ANGLE   1 
#define NBR_ANGLEr  1
#define NBR_ANGLErr 2
#define NBR_ANGLEj  3
#define NBR_ANGLEjr 5
#define NBR_ANGLEb  3
#define NBR_ANGLEbr 5
float tabDistance[NBR_DISTANCE]={45};
float tabDistancer[NBR_DISTANCEr]={220};
float tabDistancerr[NBR_DISTANCErr]={-15,280};
float tabDistancej[NBR_DISTANCEj]={0,40,220};
float tabDistancejr[NBR_DISTANCEjr]={-15,205,40,75};
float tabDistanceb[NBR_DISTANCEb]={0,40,220};
float tabDistancebr[NBR_DISTANCEbr]={-15,205,40,75};
int tabAngle[NBR_ANGLE]={0};
int tabAngler[NBR_ANGLEr]={0};
int tabAnglerr[NBR_ANGLErr]={0,180};
int tabAnglej[NBR_ANGLEj]={90,-90,0};
int tabAnglejr[NBR_ANGLEjr]={0,180,90,-90,0};
int tabAngleb[NBR_ANGLEbr]={-90,90,0};
int tabAnglebr[NBR_ANGLEbr]={0,180,-90,90,0};
int tmpValueTab;
bool parcourSens; // 0 = go // 1 = back

bool executionStepParcours(uint16_t distance, int angle);
void stopMotor();
void movingFowardRobot(uint16_t distance);
void turnedRobot(int angle);
void reverseTab(int *tab, uint16_t sizeTab, bool sens);
float accelerationDecelerationPID(float pourcentageVitesse, uint8_t acceleration, uint8_t deceleration, 
float maxSpeed, float speed);

int step = 1;
//capteur couleur
Adafruit_TCS34725 capteur = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);      // création de l'objet capteur 
void porterBalle();
void followLine();
void detectBowlingPin();
float getDistanceToBowlingPin();
float getAngleToBowlingPin();
void listenToSound();
void killBowlingPin(float distance, float angle);
void findExitLines();
int  detectColour();
void bringToRightColour();
void stopMotor();
void servoMoteur(int angle);
/* void init_Detection5kHz();*/
void detectSound();

void setup() {
  BoardInit(); 
  readEncoder0 = 0;
  readEncoder1 = 0;
  servoMoteur(180);
  SOFT_TIMER_SetCallback(FOLLOW_LINE_TIMER, followLine);
  SOFT_TIMER_SetCallback(DETECT_BOWLING_PIN_TIMER, detectBowlingPin);
  SOFT_TIMER_SetCallback(DETECT_SOUND_TIMER, detectSound);

  
  
  //Ajout du temps de répétition
  //SOFT_TIMER_SetDelay(int id, int ms)
  SOFT_TIMER_SetDelay(FOLLOW_LINE_TIMER, 50);
  SOFT_TIMER_SetDelay(DETECT_BOWLING_PIN_TIMER, 100);
  SOFT_TIMER_SetDelay(DETECT_SOUND_TIMER, 50);



  //Ajout du nombre de fois qu'il répétera la fonction (-1 = infini);
  //SOFT_TIMER_SetRepetition(int id, int nbFois)
  SOFT_TIMER_SetRepetition(FOLLOW_LINE_TIMER, -1);
  SOFT_TIMER_SetRepetition(DETECT_BOWLING_PIN_TIMER, -1);
  SOFT_TIMER_SetRepetition(DETECT_SOUND_TIMER, -1);

  //détermine si le timer pour la fonction et activé ou désactivé.
  //SOFT_TIMER_Enable(int id);
  //SOFT_TIMER_disable(int id);
  SOFT_TIMER_Enable(FOLLOW_LINE_TIMER);
  /* SOFT_TIMER_Enable(DETECT_BOWLING_PIN_TIMER);
  SOFT_TIMER_Enable(DETECT_SOUND_TIMER); */
}


void loop() {
  SOFT_TIMER_Update();
}

#pragma region BowlingPinArc
void followLine(){
  // Ajustement des roues pour le suiveur de ligne
  // Vue qu'on se base sur les capteurs, nous n'utilisons pas de PID :)
  //on va chercher le voltage
  //on ajuste les roues dépendant du voltage
  float voltageValue = (analogRead(A0))*(5/1023.0);
  //Serial.println(voltageValue); TEST
  
  float motor_left_speed = 0;
  float motor_right_speed = 0;
  
  if(voltageValue >= LINE_LEFT - 0.05 && voltageValue <= LINE_LEFT + 0.05 ){
    //Tourne vers droite
    Serial.println("LINE_LEFT");
    /* motor_left_speed = WHEEL_BASE_SPEED + (WHEEL_ADD_SPEED*2);
    motor_right_speed = WHEEL_BASE_SPEED - (WHEEL_ADD_SPEED*2); */
  } else if (voltageValue >= LINE_LEANING_LEFT - 0.05 && voltageValue <= LINE_LEANING_LEFT + 0.05 ) {
    //Tourne vers droite
    Serial.println("LINE_LEANING_LEFT");
    /* motor_left_speed = WHEEL_BASE_SPEED + WHEEL_ADD_SPEED;
    motor_right_speed = WHEEL_BASE_SPEED - WHEEL_ADD_SPEED; */
  } else if (voltageValue >= LINE_SLIGHTLY_LEFT - 0.05 && voltageValue <= LINE_SLIGHTLY_LEFT + 0.05 ) {
    //Tourne vers droite
    Serial.println("LINE_SLIGHTLY_LEFT");
    /* motor_left_speed = WHEEL_BASE_SPEED + WHEEL_ADD_SPEED;
    motor_right_speed = WHEEL_BASE_SPEED; */

  } else if (voltageValue >= LINE_CENTER - 0.05 && voltageValue <= LINE_CENTER + 0.05 ) {
    //OK good! (On fait rien)
    Serial.println("LINE_CENTER");
    /* motor_left_speed = WHEEL_BASE_SPEED;
    motor_right_speed = WHEEL_BASE_SPEED; */
  
  } else if (voltageValue >= LINE_SLIGHTLY_RIGHT - 0.05 && voltageValue <= LINE_SLIGHTLY_RIGHT + 0.05 ) {
    //Tourne vers gauche
    Serial.println("LINE_SLIGHTLY_RIGHT");
    /* motor_left_speed = WHEEL_BASE_SPEED;
    motor_right_speed = WHEEL_BASE_SPEED + WHEEL_ADD_SPEED; */
  } else if (voltageValue >= LINE_LEANING_RIGHT - 0.05 && voltageValue <= LINE_LEANING_RIGHT + 0.05 ) {
    //Tourne vers gauche     
    Serial.println("LINE_LEANING_RIGHT");
    /* motor_left_speed = WHEEL_BASE_SPEED - WHEEL_ADD_SPEED;
    motor_right_speed = WHEEL_BASE_SPEED + WHEEL_ADD_SPEED; */
  } else if (voltageValue >= LINE_RIGHT - 0.05 && voltageValue <= LINE_RIGHT + 0.05 ){
    //Tourne vers gauche
    Serial.println("LINE_RIGHT");
    /* motor_left_speed = WHEEL_BASE_SPEED - (WHEEL_ADD_SPEED*2);
    motor_right_speed = WHEEL_BASE_SPEED + (WHEEL_ADD_SPEED*2); */
  }
 
  /* MOTOR_SetSpeed(RIGHT_WHEEL , motor_right_speed);
  MOTOR_SetSpeed(LEFT_WHEEL , motor_left_speed); */
}

void detectBowlingPin(){
  //Mesurer avec l'infrarouge ou le sonar
  Serial.println("Detect Bowling Pin");
  //Prend en note à chaques fois qu'elle le détecte
}

int detectcolor(){
    uint16_t clear, red, green, blue;
    char couleur[1];
    capteur.begin();
    delay(250);                                                         // takes 50ms to read
    capteur.getRawData(&red, &green, &blue, &clear);

    if ( red>200 && green>200 && blue>100) {
        couleur[0]='j';                                                 //test
        Serial.print(" \ncouleur\t "); Serial.print(couleur);           //test
        return 1;}
    else   if ( red>200 && green>100 && blue<green) {
        couleur[0]='r';                                                 //test
        Serial.print(" \ncouleur\t "); Serial.print(couleur);           //test
        return 2;}
    else {
        couleur[0]='b';                                                 //test
        Serial.print(" \ncouleur\t "); Serial.print(couleur);           //test
        return 3;}
}

void porterBalle()
{

  int couleur = detectcolor();
  for(int i = 0; NBR_DISTANCE > i; i++){
  executionStepParcours(tabDistance[i], tabAngle[i]);
  }
  delay(500);
  servoMoteur(0);
  delay(500);
  if (couleur==1){
  for(int i = 0; NBR_DISTANCEj > i; i++){
    executionStepParcours(tabDistancej[i], tabAnglej[i]);
  }
  delay(500);
  servoMoteur(180);
  delay(500);
  for(int i = 0; NBR_DISTANCEjr > i; i++){
    executionStepParcours(tabDistancejr[i], tabAnglejr[i]);
  }
  }
    if (couleur==2){
  for(int i = 0; NBR_DISTANCEr > i; i++){
    executionStepParcours(tabDistancer[i], tabAngler[i]);
  }
  delay(500);
  servoMoteur(180);
  delay(500);
  for(int i = 0; NBR_DISTANCErr > i; i++){
    executionStepParcours(tabDistancerr[i], tabAnglerr[i]);
  }
  }  if (couleur==3){
  for(int i = 0; NBR_DISTANCEb > i; i++){
    executionStepParcours(tabDistanceb[i], tabAngleb[i]);
  }
  delay(500);
  servoMoteur(180);
  delay(500);
  for(int i = 0; NBR_DISTANCEbr > i; i++){
    executionStepParcours(tabDistancebr[i], tabAnglebr[i]);
  }
  }
}
void servoMoteur(int angle){
  SERVO_Enable(0);
  SERVO_Enable(1);
  SERVO_SetAngle(0,angle);
  SERVO_SetAngle(0,angle);
}

bool executionStepParcours(uint16_t distance, int angle){

  movingFowardRobot(distance);
  delay(75);

  turnedRobot(angle);
  delay(75);

  Serial.println(" ");
  Serial.println("----------------------------------------------------------------------");
  Serial.println(" ");
  return 1;
}

void stopMotor(){
  MOTOR_SetSpeed(RIGHT_WHEEL , 0);
  MOTOR_SetSpeed(LEFT_WHEEL , 0);
}


void movingFowardRobot(uint16_t distance){
  
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

    float pourcentageVitesse = float(readEncoder0/distanceEncodeur) * 100; 

    if(distance > 80)
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

  Serial.println(distance);
  Serial.println(distanceEncodeur);
  Serial.println(readEncoder0);
  Serial.println(readEncoder1);
  Serial.println(" ");
}



void turnedRobot(int angle){

  uint8_t angleTmp;
  if(angle < 0)
    angleTmp = angle * -1;
  else
    angleTmp = angle;

  distanceEncodeur = (((ROBOT_CIRCUMFERENCE*angleTmp)/360)/RESOLUTION_ENCODER);

  readEncoder0 = 0;
  readEncoder1 = 0;
  motorLeft = 0;
  motorRight = 0;

  ENCODER_Reset(RIGHT_WHEEL);
  ENCODER_Reset(LEFT_WHEEL);

  while(((distanceEncodeur) > abs(readEncoder0)) && ((distanceEncodeur) > abs(readEncoder1))){ //les 2 roue tournes
     
    motorLeft = SPEED_MIN;

    //PID FOR ENCODER
    float error = abs(readEncoder0) - abs(readEncoder1);
    float p = error * KP_TURNING;

    sumError += error;
    float i = sumError * KI_FOWARD;

    float adjSpeed = p + i;
    motorRight = motorLeft + adjSpeed;

    if(angle > 0){ //test si l'angle est negative, tourne vers la droite
      motorLeft = -1 * motorLeft;
    }
    else if(angle < 0){ //test si l'angle est positive, tourne vers la gauche
      motorRight = -1 * motorRight;
    }

    MOTOR_SetSpeed(LEFT_WHEEL, motorLeft);
    MOTOR_SetSpeed(RIGHT_WHEEL, motorRight);

    readEncoder0 = ENCODER_Read(LEFT_WHEEL);  
    readEncoder1 = ENCODER_Read(RIGHT_WHEEL);

  }

  stopMotor();

  Serial.println(angle);
  Serial.println(distanceEncodeur);
  Serial.println(readEncoder0);
  Serial.println(readEncoder1);
  Serial.println(" ");
}


void reverseTab(int *tab, uint16_t sizeTab, bool sens){

  if(sens){

    tmpValueTab = -1 * tab[sizeTab - 1];

    for(int i = sizeTab - 1; i > 0; i--)
      tab[i] = -1 * tab[i - 1];
    
    tab[0] = tmpValueTab;
  }
  else{
    tmpValueTab = -1 * tab[0];

    for(uint16_t i = 0; i < sizeTab; i++)
      tab[i] = -1 * tab[i + 1];
    
    tab[sizeTab - 1] = tmpValueTab;
  }
}


float accelerationDecelerationPID(float pourcentageVitesse, uint8_t acceleration, uint8_t deceleration, float maxSpeed, float speed){

  float _speed = 0;
  float dAcceleration = maxSpeed/acceleration;
  float dDeceleration = -maxSpeed/(100-deceleration);
  float b = maxSpeed-(dDeceleration*deceleration);//pente de la fonction de decrementation


  if(acceleration >= pourcentageVitesse){ // de 0 a ACCELERATION%
    _speed = pourcentageVitesse * dAcceleration;

    if(_speed < SPEED_MIN)
      _speed = SPEED_MIN;
    if(_speed >= maxSpeed)
      _speed = maxSpeed;

    return _speed;
  }
  else if(deceleration <= pourcentageVitesse){ // de DECELERATION a 100%
    _speed = dDeceleration * pourcentageVitesse + b;

    if(_speed < SPEED_MIN)
      _speed = SPEED_MIN;

    return _speed;
  }
  else{
    return speed;
  }
}
void detectSound(){
  float voltageValue = analogRead(A1) *(5/1023.0);
  if(false /*Valeur qu'on voudra */){
     SOFT_TIMER_Disable(FOLLOW_LINE_TIMER);
     stopMotor();
  }
}

/* void init_Detection5kHz(){
  PORTK |= (1<<7); //active PULL_DOWN A15
  PCICR |= (1<<2); //enable PCINT 23
  PCMSK2 |= (1<<7);
  sei();
}

ISR(PCINT2_vect){
  SOFT_TIMER_Disable(FOLLOW_LINE_TIMER);

} */

#pragma endregion

#pragma region ColourDetection
#pragma endregion