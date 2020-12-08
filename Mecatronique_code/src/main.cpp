#include <Arduino.h>

//Définitions du moteur droit
#define left_pin_PWM 6
#define left_pin_dir_a 11
#define left_pin_dir_b 12
#define left_pin_interrupt 3

//Définitions du moteur gauche
#define right_pin_PWM 5
#define right_pin_dir_a 9
#define right_pin_dir_b 10
#define right_pin_interrupt 2

//Définitions de l'ultrason droit
#define right_pin_trigger_ultrasound 7
#define right_pin_echo_ultrasound 8

//Définitions de l'ultrason gauche 
#define left_pin_trigger_ultrasound 9
#define left_pin_echo_ultrasound 10

#define taille_roues 


bool led_status = false;
bool Rdirection = false; //false = en avant
bool Ldirection = Rdirection;
volatile int Rtick = 0;
volatile int Ltick = 0;


double target_rpm = 120.00; //Valeur fixe
double Rrpm = target_rpm; //Valeur changeante right
double Lrpm = target_rpm; //Valeur changeante left
double nb_tick_rota = 224.4;
const int del = 33; //Delay en millisec
const double del_min = 60000.00/del; //delay en minutes

double Rvitesse; //Vitesse en rpm droite
double Lvitesse; //L

double Rdistance = 0.00;
double Ldistance = 0.00;

//Définitions du PID moteur droit
double Rerror; //Erreur proportionnelle instantanée
double RError_integrale = 0; //Erreur cumulée (intégrale)
double Rderror; //Erreur dérivative (delta erreur)
double Rold_error = 120; //Erreur du cycle précédent

//Définitions du PID moteur droit
double Lerror; //Erreur proportionnelle instantanée
double LError_integrale = 0; //Erreur cumulée (intégrale)
double Lderror; //Erreur dérivative (delta erreur)
double Lold_error = 120; //Erreur du cycle précédent

#define kp 0.1
#define ki 0.52
#define kd 0.002	
double RPID;
double LPID;

bool Remergency_break(int target){
  digitalWrite(right_pin_trigger_ultrasound,HIGH);
  delayMicroseconds(10);
  digitalWrite(right_pin_trigger_ultrasound, LOW);
  float distance = (pulseIn(right_pin_echo_ultrasound,HIGH)/2000000.00) * 342000;
  digitalWrite(left_pin_trigger_ultrasound,HIGH);
  delayMicroseconds(10);
  digitalWrite(left_pin_trigger_ultrasound, LOW);
  if(distance>(pulseIn(left_pin_echo_ultrasound,HIGH)/2000000.00) * 342000)
  {
  distance = pulseIn(left_pin_echo_ultrasound,HIGH)/2000000.00;
  }
  Serial.println(distance);
  
  return  (distance <=150.00) ? 0.00 : target;
}

void EmergencyBreak(int target) {
  if (Remergency_break(target) == 0.00) {
    Ldirection != Ldirection;
    Rdistance = 0.00;
    while (Rdistance <= 55) {
      cli();
      Rrpm = target_rpm;
      Lrpm = target_rpm;

      Rvitesse = Rtick/nb_tick_rota; //On obtient le nombre de tick effectué depuis dernière loop
      Rdistance += Rvitesse;
      
      Rvitesse *= del_min; //On passe d'une distance à une vitesse

      Lvitesse = Ltick/nb_tick_rota;
      Ldistance += Lvitesse;
      Lvitesse *= del_min;
      Rerror = Rrpm - Rvitesse;
      Lerror = Lrpm - Lvitesse;
        
      //Serial.println(error);
      //PID
      RError_integrale += Rerror;
      LError_integrale += Lerror;

      Rderror = Rerror - Rold_error;
      Lderror = Lerror - Lold_error;

      RError_integrale = (RError_integrale>10 )? 10 : RError_integrale; //Sécurités pour éviter valeurs aberrantes
      RError_integrale = (RError_integrale< -10 )? -10 : RError_integrale;

      LError_integrale = (LError_integrale>10) ? 10 : LError_integrale; 
      LError_integrale = (LError_integrale<-10) ? -10 : LError_integrale;

      RPID = kp*Rerror + ki*RError_integrale + kd*Rderror;
      RPID = (RPID>5) ? 5: RPID;
      RPID = (RPID<0) ? 0 : RPID;
      //Serial.println(PID*51);

      LPID = kp*Lerror + ki*LError_integrale + kd*Lderror;
      LPID = (LPID>5) ? 5: LPID;
      LPID = (LPID<0) ? 0 : LPID;
      //Serial.println(LPID*51);
      //Le PID calculé est en valeur de tension, nous le voulons en byte pour le PWM
      
      //Serial.println(vitesse);
      analogWrite(right_pin_PWM,(int)(RPID*51));
      analogWrite(left_pin_PWM,(int)(LPID*51));
      Rtick = 0;
      Ltick = 0;
      Rold_error = Rerror;
      Lold_error = Lerror;

      
      sei();
    }
  }
  Ldirection != Ldirection;
}

// bool Lemergency_break(int target) {
//   digitalWrite(left_pin_trigger_ultrasound,HIGH);
//   delayMicroseconds(10);
//   digitalWrite(left_pin_trigger_ultrasound, LOW);
//   float distance = (pulseIn(left_pin_echo_ultrasound,HIGH)/2000000.00) * 342000;
//   Serial.println(distance);
//   return  (distance <=50.00) ? 0.00 : target;
// }

void Lcounter(){
  Ltick++;
  
}

void Rcounter(){
  Rtick++;
}

ISR(TIMER1_COMPA_vect){
  
  cli();
  Rrpm = target_rpm;
  Lrpm = target_rpm;

  Rvitesse = Rtick/nb_tick_rota; //On obtient le nombre de tick effectué depuis dernière loop
  Rdistance += Rvitesse;
  
  Rvitesse *= del_min; //On passe d'une distance à une vitesse

  Lvitesse = Ltick/nb_tick_rota;
  Ldistance += Lvitesse;
  Lvitesse *= del_min; //
  //Rrpm =  Remergency_break(target_rpm); 
  //Lrpm =  Remergency_break(target_rpm);
  EmergencyBreak();
  Rerror = Rrpm - Rvitesse;
  Lerror = Lrpm - Lvitesse;
    
  //Serial.println(error);
  //PID
  RError_integrale += Rerror;
  LError_integrale += Lerror;

  Rderror = Rerror - Rold_error;
  Lderror = Lerror - Lold_error;

  RError_integrale = (RError_integrale>10 )? 10 : RError_integrale; //Sécurités pour éviter valeurs aberrantes
  RError_integrale = (RError_integrale< -10 )? -10 : RError_integrale;

  LError_integrale = (LError_integrale>10) ? 10 : LError_integrale; 
  LError_integrale = (LError_integrale<-10) ? -10 : LError_integrale;

  RPID = kp*Rerror + ki*RError_integrale + kd*Rderror;
  RPID = (RPID>5) ? 5: RPID;
  RPID = (RPID<0) ? 0 : RPID;
  //Serial.println(PID*51);

  LPID = kp*Lerror + ki*LError_integrale + kd*Lderror;
  LPID = (LPID>5) ? 5: LPID;
  LPID = (LPID<0) ? 0 : LPID;
  //Serial.println(LPID*51);
  //Le PID calculé est en valeur de tension, nous le voulons en byte pour le PWM
  
  //Serial.println(vitesse);
  analogWrite(right_pin_PWM,(int)(RPID*51));
  analogWrite(left_pin_PWM,(int)(LPID*51));
  Rtick = 0;
  Ltick = 0;
  Rold_error = Rerror;
  Lold_error = Lerror;

  
  sei();
}

void setup(){

  Serial.begin(9600);
  //Pin mode des pins du moteur droit
  pinMode(right_pin_PWM, OUTPUT);
  pinMode(right_pin_dir_a,OUTPUT);
  pinMode(right_pin_dir_b,OUTPUT);
  pinMode(right_pin_interrupt,INPUT);

  //Pin mode des pins du moteur gauche
  pinMode(left_pin_PWM, OUTPUT);
  pinMode(left_pin_dir_a,OUTPUT);
  pinMode(left_pin_dir_b,OUTPUT);
  pinMode(left_pin_interrupt,INPUT);

  pinMode(LED_BUILTIN,OUTPUT);
  pinMode(right_pin_trigger_ultrasound,OUTPUT);
  pinMode(right_pin_echo_ultrasound,INPUT);

  pinMode(left_pin_trigger_ultrasound,OUTPUT);
  pinMode(left_pin_echo_ultrasound,INPUT);

  //Set default values for right motor
  digitalWrite(right_pin_PWM,0);
  digitalWrite(right_pin_dir_a,Rdirection);
  digitalWrite(right_pin_dir_b,!Rdirection);
  attachInterrupt(digitalPinToInterrupt(right_pin_interrupt) ,Rcounter,RISING);

  //Set default values for left motor
  digitalWrite(left_pin_PWM,0);
  digitalWrite(left_pin_dir_a,!Ldirection);
  digitalWrite(left_pin_dir_b,Ldirection);
  attachInterrupt(digitalPinToInterrupt(left_pin_interrupt) ,Lcounter,RISING);
  


  // TIMER 1 for interrupt frequency 30.00120004800192 Hz:
  cli(); // stop interrupts
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  TCNT1  = 0; // initialize counter value to 0
  // set compare match register for 30.00120004800192 Hz increments
  OCR1A = 8332; // = 16000000 / (64 * 30.00120004800192) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12, CS11 and CS10 bits for 64 prescaler
  TCCR1B |= (0 << CS12) | (1 << CS11) | (1 << CS10);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei(); // allow interrupts
}



void loop(){
  
  //Pas nécessaire d'avoir une void loop
 

}