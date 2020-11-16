#include <Arduino.h>
const float P = 0.095;
const float I = 0.66;
const float D = 0.0002;

const int pinVitesseVoulue = 6;
const int pinVitesseMesuree = 9;
const int vitesseMoteurMax = 118;
int vitesseMoteurVoulue;

void setup() {
  // put your setup code here, to run once:
  pinMode(pinVitesseVoulue, OUTPUT);
  pinMode(pinVitesseMesuree, INPUT);
  
  Serial.print("Entrez la vitesse du moteur voulue");
  vitesseMoteurVoulue = Serial.read();
}

void loop() {
  // put your main code here, to run repeatedly:
  //List<float> erreur = new List<float>();
  float vitesseMesuree = (float)analogRead(pinVitesseMesuree) * ((float)vitesseMoteurMax/255);

  analogWrite(pinVitesseVoulue,255); //L'arduino envoie 5v en PWM, le moteur tourne à sa vitesse maximum
  digitalWrite(12,LOW);//On tourne dans un sens
  digitalWrite(13,HIGH);
  delay(2000); // On attend 2 secondes
  digitalWrite(12,HIGH); //On tourne dans l'autre sens
  digitalWrite(13,LOW);
  delay(2000);//On attend 2 secondes

}