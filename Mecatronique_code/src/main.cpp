#include <Arduino.h>

#define pin_PWM 6
#define pin_dir_a 12
#define pin_dir_b 11
#define pin_interrupt 2

bool led_status = false;
bool direction = true; //true = en avant
volatile int tick = 0;


double target_rpm = 100.00;
double nb_tick_rota = 224.4;
const int del = 50; //Delay en millisec
const double del_min = 60000.00/del; //delay en minutes
double vitesse; //Vitesse en rpm

double error; //Erreur proportionnelle instantanée
double Error_integrale = 0; //Erreur cumulée (intégrale)
double derror; //Erreur dérivative (delta erreur)
double old_error = 120; //Erreur du cycle précédent

#define kp 0.3
#define ki 0.3
#define kd 0.0
double PID;

void counter(){
  tick++;
}


void setup(){

  Serial.begin(9600);
  pinMode(pin_PWM, OUTPUT);
  pinMode(pin_dir_a,OUTPUT);
  pinMode(pin_dir_b,OUTPUT);
  pinMode(pin_interrupt,INPUT);
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(pin_PWM,0);
  digitalWrite(pin_dir_a,direction);
  digitalWrite(pin_dir_b,!direction);
  attachInterrupt(digitalPinToInterrupt(pin_interrupt) ,counter,RISING);
}



void loop(){
  
  
  
  delay(del);
  cli();
  vitesse = tick/nb_tick_rota; //On obtient le nombre de tick effectué depuis dernière loop
  
  vitesse *= del_min; //On passe d'une distance à une vitesse
  
  error = target_rpm - vitesse;
  //Serial.println(error);
  Error_integrale += error;
  derror = error - old_error;
  Error_integrale = (Error_integrale>10 )? 10 : Error_integrale; //Sécurités pour éviter valeurs aberrantes
  Error_integrale = (Error_integrale< -10 )? -10 : Error_integrale;
  PID = kp*error + ki*Error_integrale + kd*derror;
  PID = (PID>5) ? 5: PID;
  PID = (PID<0) ? 0 : PID;
 
  //Le PID calculé est en valeur de tension, nous le voulons en byte pour le PWM
  PID = (int)(PID*51);
  Serial.println(vitesse);
  analogWrite(pin_PWM,PID);
  tick = 0;
  old_error = error;
  
  sei();
  
 

}