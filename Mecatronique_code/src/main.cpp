#include <Arduino.h>
#include <TimerOne.h>

const float P = 0.095;
const float I = 0.66;
const float D = 0.0002;

const int pin_pwr = 6;
// const int pinVitesseMesuree = 9;
const int pin_encoder = 2;
// const int pinEncodeur2 = 11;
// const int vitesseMoteurMax = 118;
int vitesseMoteurVoulue = 20;
// float erreur = 0;
// float erreurPrecedente;
// float sommeErreur = 0;
int tick = 0;

// int interrupt_Pin = 2;

// void setup()
// {
//   // put your setup code here, to run once:
//   pinMode(pinVitesseVoulue, OUTPUT);
//   pinMode(pinVitesseMesuree, INPUT);
//   pinMode(12,OUTPUT);
//   pinMode(13,OUTPUT);
//   pinMode(pinEncodeur1,INPUT);
//   pinMode(pinEncodeur2,INPUT);
//   pinMode(interrupt_Pin,INPUT_PULLUP);
//   Serial.begin(9600);
//   //Serial.print("Entrez la vitesse du moteur voulue");
//   vitesseMoteurVoulue = 255;
// }

// void counter(){
//   nTicks++;
// }

// void loop()
// {
//   // put your main code here, to run repeatedly:
//   unsigned long time = millis();
//   int prevTime = time;
//   nTicks = 0;
//   digitalWrite(12, HIGH);
//   digitalWrite(13, LOW);
//  Serial.println(digitalRead(time - prevTime));
//   while (time - prevTime < 10) {
//     time = millis();
//     attachInterrupt(digitalPinToInterrupt(interrupt_Pin) ,counter,RISING);
//     Serial.println(digitalRead(time - prevTime));
//     //Serial.println(nTicks);
//     //Serial.println("Je suis dedans");
    
//   }
  
//   float vitesseMesuree = (2*3.1415)/(20*0.01*nTicks);
//   //Serial.println(vitesseMesuree);
//   erreurPrecedente = erreur;
//   erreur = vitesseMesuree - vitesseMoteurVoulue;
//   sommeErreur += erreur;


//   //float nouvelleVitesse = (P * erreur + D * Math.Abs(erreur - erreurPrecedente) + I * sommeErreur) * 255/118;
//   //Serial.println(nouvelleVitesse);
//   analogWrite(pinVitesseVoulue, 255);
//   // analogWrite(pinVitesseVoulue, 255); //L'arduino envoie 5v en PWM, le moteur tourne à sa vitesse maximum
//   // digitalWrite(12, LOW);              //On tourne dans un sens
//   // digitalWrite(13, HIGH);
//   // delay(2000);            // On attend 2 secondes
//   // digitalWrite(12, HIGH); //On tourne dans l'autre sens
//   // digitalWrite(13, LOW);
//   // delay(2000); //On attend 2 secondes
// }

void encoderInterrupt(){
  tick++;
}
void Cycle()///called by the timer
{
  cli(); //éteint les interrupts

    //calculate error and pid
    //float vitesseMesuree = (2*3.1415)/(20*0.01*nTicks);
    int target_ticks = (2*3.1415)/(20*0.01*vitesseMoteurVoulue);
    int e = target_ticks - tick;
    int olde = e;
    int E = E+e;
    int de = e-olde;
    long PID_ = (P*e)+(I*E)+(D*de);
    //long mapped = map(PID_,0,1000,0,1023);
    analogWrite(pin_pwr,PID_);
    //reset
    

    tick = 0;
    
  
  sei(); //relance les interrupts
}
void setup(void)
{
  Serial.begin(9600);
  unsigned long period = 100000;
  //TIMER initialization
  Timer1.initialize(period); //initialisation du timer
  Timer1.attachInterrupt(Cycle); //attachInterrupt
  //pin init
  pinMode(pin_pwr,OUTPUT);
  pinMode(12,OUTPUT);
  pinMode(13,OUTPUT);
  digitalWrite(12, HIGH);
  digitalWrite(13, LOW);
  pinMode(LED_BUILTIN,OUTPUT);
  pinMode(pin_encoder,INPUT);
  //encoder initialisation
  attachInterrupt(digitalPinToInterrupt(pin_encoder),encoderInterrupt,RISING); //! slow must be changed to attachInterruptVector
  //attachInterruptVector(,encoderInterrupt,RISING);
  /*uint32_t mask = (0x09 << 16) | 0x01000000;//setup mask for rising edge
  volatile uint32_t *config;
  config = portConfigRegister(pin_encoder);
  */
  //setting up pwm precision
  // analogWriteFrequency(pin_pwr,F_CPU/1E6);//setting up ideal frequency pedending on cpu frequency
  // analogWriteResolution(10); // 0 - 255
}

void loop(void) ///main looppin_dir1
{
  Serial.println((2*3.1415)/(20*0.01*tick));
 
}



