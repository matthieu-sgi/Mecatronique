#include <Arduino.h>

#define pin_PWM 6
#define pin_dir_a 12
#define pin_dir_b 11
#define pin_interrupt 2
<<<<<<< HEAD
#define pin_trigger_ultrasound 7
#define pin_echo_ultrasound 8
bool led_status = false;
=======

//bool led_status = false;
>>>>>>> c9d7bb708886a861ab2d45d1bb734895f58fef20
bool direction = true; //true = en avant
volatile int tick = 0;


double target_rpm = 120.00;
double rpm = target_rpm;
double nb_tick_rota = 224.4;
const int del = 33; //Delay en millisec
const double del_min = 60000.00/del; //delay en minutes
double vitesse; //Vitesse en rpm

double error; //Erreur proportionnelle instantanée
double Error_integrale = 0; //Erreur cumulée (intégrale)
double derror; //Erreur dérivative (delta erreur)
double old_error = 120; //Erreur du cycle précédent

#define kp 0.1
#define ki 0.52
#define kd 0.002	
double PID;

float emergency_break(int target){
  digitalWrite(pin_trigger_ultrasound,HIGH);
  delayMicroseconds(10);
  digitalWrite(pin_trigger_ultrasound, LOW);
  float distance = (pulseIn(pin_echo_ultrasound,HIGH)/2000000.00) * 342000;
  Serial.println(distance);
  return  (distance <=50.00) ? 0.00 : target;
}

void counter(){
  tick++;
}

ISR(TIMER1_COMPA_vect){
  
  cli();
  rpm = target_rpm;
  vitesse = tick/nb_tick_rota; //On obtient le nombre de tick effectué depuis dernière loop
  
  vitesse *= del_min; //On passe d'une distance à une vitesse
<<<<<<< HEAD
  rpm =  emergency_break(target_rpm); 
  error = rpm - vitesse;
    
  //Serial.println(error);
=======
  
  error = target_rpm - vitesse;

>>>>>>> c9d7bb708886a861ab2d45d1bb734895f58fef20
  Error_integrale += error;
  derror = error - old_error;
  Error_integrale = (Error_integrale>10 )? 10 : Error_integrale; //Sécurités pour éviter valeurs aberrantes
  Error_integrale = (Error_integrale< -10 )? -10 : Error_integrale;
  PID = kp*error + ki*Error_integrale + kd*derror;
  PID = (PID>5) ? 5: PID;
  PID = (PID<0) ? 0 : PID;
  
 
  //Le PID calculé est en valeur de tension, nous le voulons en byte pour le PWM
  
  //Serial.println(vitesse);
  analogWrite(pin_PWM,(int)(PID*51));
  tick = 0;
  old_error = error;
  
  sei();
}

void setup(){

  Serial.begin(9600);
  pinMode(pin_PWM, OUTPUT);
  pinMode(pin_dir_a,OUTPUT);
  pinMode(pin_dir_b,OUTPUT);
  pinMode(pin_interrupt,INPUT);
<<<<<<< HEAD
  pinMode(LED_BUILTIN,OUTPUT);
  pinMode(pin_trigger_ultrasound,OUTPUT);
  pinMode(pin_echo_ultrasound,INPUT);
=======
  //pinMode(LED_BUILTIN,OUTPUT);
>>>>>>> c9d7bb708886a861ab2d45d1bb734895f58fef20
  digitalWrite(pin_PWM,0);
  digitalWrite(pin_dir_a,direction);
  digitalWrite(pin_dir_b,!direction);

  attachInterrupt(digitalPinToInterrupt(pin_interrupt) ,counter,RISING);


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