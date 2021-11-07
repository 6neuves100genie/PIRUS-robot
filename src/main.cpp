
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

#define FOLLOW_LINE_TIMER 1
#define DETECT_BOWLING_PIN_TIMER 2
#define LISTEN_SOUND_TIMER 3

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
void servoMoteur(int angle);

void setup() {
  BoardInit();
  capteur.begin(); 

  SOFT_TIMER_SetCallback(FOLLOW_LINE_TIMER, followLine);
  SOFT_TIMER_SetCallback(DETECT_BOWLING_PIN_TIMER, detectBowlingPin);
  SOFT_TIMER_SetCallback(LISTEN_SOUND_TIMER, listenToSound);

  SOFT_TIMER_SetDelay(FOLLOW_LINE_TIMER, 50);
  SOFT_TIMER_SetDelay(DETECT_BOWLING_PIN_TIMER, 100);
  SOFT_TIMER_SetDelay(LISTEN_SOUND_TIMER, 100);

  SOFT_TIMER_SetRepetition(FOLLOW_LINE_TIMER, -1);
  SOFT_TIMER_SetRepetition(DETECT_BOWLING_PIN_TIMER, -1);
  SOFT_TIMER_SetRepetition(LISTEN_SOUND_TIMER, -1);

  SOFT_TIMER_Enable(FOLLOW_LINE_TIMER);
  SOFT_TIMER_Enable(DETECT_BOWLING_PIN_TIMER);
  SOFT_TIMER_Enable(LISTEN_SOUND_TIMER);
}


void loop() {
  SOFT_TIMER_Update();
}

#pragma region BowlingPinArc
void followLine(){
  //Suivre la ligne Étape 1 et probablement après avoir fait tombé la quille
  Serial.println("Follow Line");
}

void detectBowlingPin(){
  //Mesurer avec l'infrarouge ou le sonar
  Serial.println("Detect Bowling Pin");
  //Prend en note à chaques fois qu'elle le détecte
}

void listenToSound(){
  if(/*Détecte le son de 5khz*/ false){
    SOFT_TIMER_Disable(FOLLOW_LINE_TIMER);
    SOFT_TIMER_Disable(DETECT_BOWLING_PIN_TIMER);
    SOFT_TIMER_Disable(LISTEN_SOUND_TIMER);
    float distance = getDistanceToBowlingPin();
    float angle = getAngleToBowlingPin();
    killBowlingPin(distance, angle);
  }
}

int detectcolor(){
    uint16_t clear, red, green, blue;
    char couleur[1];

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

void servoMoteur(int angle){
  SERVO_Enable(0);
  SERVO_Enable(1);
  SERVO_SetAngle(0,angle);
  SERVO_SetAngle(0,angle);
}

#pragma endregion

#pragma region ColourDetection
#pragma endregion