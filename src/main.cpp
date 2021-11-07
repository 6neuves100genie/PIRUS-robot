
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
#define LISTEN_SOUND_TIMER 3

// OBP-704 possible values for suiveur de ligne
#define LINE_LEFT  0.25             // 1 0 0 0
#define LINE_LEANING_LEFT 0.95      // 1 1 0 0
#define LINE_SLIGHTLY_LEFT 0.78     // 0 1 0 0
#define LINE_CENTER 2.16            // 0 1 1 0
#define LINE_SLIGHTLY_RIGHT 1.45    // 0 0 1 0
#define LINE_LEANING_RIGHT 4.19     // 0 0 1 1
#define LINE_RIGHT 2.81             // 0 0 0 1
#define LINE_LOST 0.07              // 0 0 0 0


int step = 1;
Adafruit_TCS34725 capteur = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);      // création de l'objet capteur 

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

void setup() {
  BoardInit();
  capteur.begin(); 

  //Création de Timers car notre robot ne peut pas faire du multi-thread
  //SOFT_TIMER_SetCallback(int id, func);
  SOFT_TIMER_SetCallback(FOLLOW_LINE_TIMER, followLine);
  SOFT_TIMER_SetCallback(DETECT_BOWLING_PIN_TIMER, detectBowlingPin);
  SOFT_TIMER_SetCallback(LISTEN_SOUND_TIMER, listenToSound);
  
  
  //Ajout du temps de répétition
  //SOFT_TIMER_SetDelay(int id, int ms)
  SOFT_TIMER_SetDelay(FOLLOW_LINE_TIMER, 200);
  SOFT_TIMER_SetDelay(DETECT_BOWLING_PIN_TIMER, 100);
  SOFT_TIMER_SetDelay(LISTEN_SOUND_TIMER, 100);


  //Ajout du nombre de fois qu'il répétera la fonction (-1 = infini);
  //SOFT_TIMER_SetRepetition(int id, int nbFois)
  SOFT_TIMER_SetRepetition(FOLLOW_LINE_TIMER, -1);
  SOFT_TIMER_SetRepetition(DETECT_BOWLING_PIN_TIMER, -1);
  SOFT_TIMER_SetRepetition(LISTEN_SOUND_TIMER, -1);

  //détermine si le timer pour la fonction et activé ou désactivé.
  //SOFT_TIMER_Enable(int id);
  //SOFT_TIMER_disable(int id);
  SOFT_TIMER_Enable(FOLLOW_LINE_TIMER);
  SOFT_TIMER_Enable(LISTEN_SOUND_TIMER);
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
    //Serial.println("LINE_LEFT"); TEST
    motor_left_speed = WHEEL_BASE_SPEED - (WHEEL_ADD_SPEED*2);
    motor_right_speed = WHEEL_BASE_SPEED + (WHEEL_ADD_SPEED*2);
  } else if (voltageValue >= LINE_LEANING_LEFT - 0.05 && voltageValue <= LINE_LEANING_LEFT + 0.05 ) {
    //Tourne vers droite
    //Serial.println("LINE_LEANING_LEFT"); TEST
    motor_left_speed = WHEEL_BASE_SPEED - WHEEL_ADD_SPEED;
    motor_right_speed = WHEEL_BASE_SPEED + WHEEL_ADD_SPEED;
  } else if (voltageValue >= LINE_SLIGHTLY_LEFT - 0.05 && voltageValue <= LINE_SLIGHTLY_LEFT + 0.05 ) {
    //Tourne vers droite
    //Serial.println("LINE_SLIGHTLY_LEFT"); TEST
    motor_left_speed = WHEEL_BASE_SPEED;
    motor_right_speed = WHEEL_BASE_SPEED + WHEEL_ADD_SPEED;

  } else if (voltageValue >= LINE_CENTER - 0.05 && voltageValue <= LINE_CENTER + 0.05 ) {
    //OK good! (On fait rien)
    //Serial.println("LINE_CENTER"); TEST
    motor_left_speed = WHEEL_BASE_SPEED;
    motor_right_speed = WHEEL_BASE_SPEED;
  
  } else if (voltageValue >= LINE_SLIGHTLY_RIGHT - 0.05 && voltageValue <= LINE_SLIGHTLY_RIGHT + 0.05 ) {
    //Tourne vers gauche
    //Serial.println("LINE_SLIGHTLY_RIGHT"); TEST
    motor_left_speed = WHEEL_BASE_SPEED + WHEEL_ADD_SPEED;
    motor_right_speed = WHEEL_BASE_SPEED;
  } else if (voltageValue >= LINE_LEANING_RIGHT - 0.05 && voltageValue <= LINE_LEANING_RIGHT + 0.05 ) {
    //Tourne vers gauche     
    //Serial.println("LINE_LEANING_RIGHT"); TEST
    motor_left_speed = WHEEL_BASE_SPEED + WHEEL_ADD_SPEED;
    motor_right_speed = WHEEL_BASE_SPEED - WHEEL_ADD_SPEED;
  } else if (voltageValue >= LINE_RIGHT - 0.05 && voltageValue <= LINE_RIGHT + 0.05 ){
    //Tourne vers gauche
    //Serial.println("LINE_RIGHT"); TEST
    motor_left_speed = WHEEL_BASE_SPEED + (WHEEL_ADD_SPEED*2);
    motor_right_speed = WHEEL_BASE_SPEED - (WHEEL_ADD_SPEED*2);
  }
 
  MOTOR_SetSpeed(RIGHT_WHEEL , motor_right_speed);
  MOTOR_SetSpeed(LEFT_WHEEL , motor_left_speed);
}

void detectBowlingPin(){
  //Mesurer avec l'infrarouge ou le sonar
  Serial.println("Detect Bowling Pin");
  //Prend en note à chaques fois qu'elle le détecte
}

void listenToSound(){
  if(/*Détecte le son de 5khz*/ false){
    SOFT_TIMER_Enable(DETECT_BOWLING_PIN_TIMER);
    SOFT_TIMER_Disable(LISTEN_SOUND_TIMER);
    float distance = getDistanceToBowlingPin();
    float angle = getAngleToBowlingPin();
    killBowlingPin(distance, angle);
  }
}

int detectcolor(){
    uint16_t clear, red, green, blue;
    char couleur[1];

//    delay(250);                                                         // takes 50ms to read
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

void servoMoteur(int angle){
  SERVO_Enable(0);
  SERVO_Enable(1);
  SERVO_SetAngle(0,angle);
  SERVO_SetAngle(0,angle);
}

void stopMotor(){
  MOTOR_SetSpeed(RIGHT_WHEEL , 0);
  MOTOR_SetSpeed(LEFT_WHEEL , 0);
}

#pragma endregion

#pragma region ColourDetection
#pragma endregion