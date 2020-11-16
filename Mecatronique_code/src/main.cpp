#include <Arduino.h>

const float P = 0.095;
const float I = 0.66;
const float D = 0.0002;

const int pinVitesseVoulue = 6;
const int pinVitesseMesuree = 9;
const int pinEncodeur1 = 10;
const int pinEncodeur2 = 11;
const int vitesseMoteurMax = 118;
int vitesseMoteurVoulue;
float erreur = 0;
float erreurPrecedente;
float sommeErreur = 0;

void setup()
{
  // put your setup code here, to run once:
  pinMode(pinVitesseVoulue, OUTPUT);
  pinMode(pinVitesseMesuree, INPUT);
  pinMode(12,OUTPUT);
  pinMode(13,OUTPUT);
  pinMode(pinEncodeur1,INPUT);
  pinMode(pinEncodeur2,INPUT);
  float erreur = 0;

  Serial.print("Entrez la vitesse du moteur voulue");
  vitesseMoteurVoulue = Serial.read();
}

void loop()
{
  // put your main code here, to run repeatedly:
  unsigned long time = millis();
  int prevTime = time;
  int nTicks = 0;
  while (time - prevTime < 10) {
    if (digitalRead(pinEncodeur1) == 255) nTicks++;
  }
  float vitesseMesuree = (2*3.1415)/(20*0.01*nTicks);
  erreurPrecedente = erreur;
  erreur = vitesseMesuree - vitesseMoteurVoulue;
  sommeErreur += erreur;

  float nouvelleVitesse = P * erreur + D * (erreur - erreurPrecedente) + I * sommeErreur;
  analogWrite(pinVitesseVoulue, nouvelleVitesse);
  // analogWrite(pinVitesseVoulue, 255); //L'arduino envoie 5v en PWM, le moteur tourne à sa vitesse maximum
  // digitalWrite(12, LOW);              //On tourne dans un sens
  // digitalWrite(13, HIGH);
  // delay(2000);            // On attend 2 secondes
  // digitalWrite(12, HIGH); //On tourne dans l'autre sens
  // digitalWrite(13, LOW);
  // delay(2000); //On attend 2 secondes
}