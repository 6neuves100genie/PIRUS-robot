
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

#define FOLLOW_LINE_TIMER 1
#define DETECT_BOWLING_PIN_TIMER 2
#define LISTEN_SOUND_TIMER 3

int step = 1;


void followLine();
void detectBowlingPin();
float getDistanceToBowlingPin();
float getAngleToBowlingPin();
void listenToSound();
void killBowlingPin(float distance, float angle);
void findExitLines();
void detectColour();
void bringToRightColour();

void setup() {
  BoardInit();

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
#pragma endregion

#pragma region ColourDetection
#pragma endregion