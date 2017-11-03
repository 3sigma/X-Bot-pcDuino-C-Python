/******************************************************************************
Firmware du robot X-Bot pcDuino C / Python, disponible à l'adresse:
http://boutique.3sigma.fr/12-robots

Auteur: 3Sigma
Version 1.0 - 01/10/2017
*******************************************************************************/

// Inclusion d'une bibliothèque permettant l'exécution à cadence fixe
// d'une partie du programme. Télécharger à l'adresse http://www.3sigma.fr/telechargements/FlexiTimer2.zip
// et décompresser dans le sous-répertoire « libraries » de votre installation Arduino
// Pour plus de détails, voir les pages (en anglais): 
// http://www.arduino.cc/playground/Main/FlexiTimer2
// https://github.com/wimleers/flexitimer2
#include <FlexiTimer2.h>

// Inclusion d'une bibliothèque permettant de lire et d'écrire plus rapidement sur les entrées-sorties digitales.
// Télécharger à l'adresse http://www.3sigma.fr/telechargements/digitalWriteFast.zip
// et décompresser dans le sous-répertoire « libraries » de votre installation Arduino
#include <digitalWriteFast.h> 

// Inclusion d'une bibliothèque permettant de gérer les interruptions externes
// et le "PinChange Interrupts"
#include <EnableInterrupt.h>

// Inclusion d'une bibliothèque pour la gestion du capteur de distance
#include <NewPing.h>

// Inclusion de la bibliothèque permettant de gérer l'i2c
#include "Slave.h"
Slave slave;

#define Nmoy 10

// Définitions et déclarations pour le codeur incrémental du moteur droit
#define codeurDroitInterruptionA 0
#define codeurDroitPinA 3
#define codeurDroitPinB 9
volatile long ticksCodeurDroit = 0;
uint8_t indiceTicksCodeurDroit = 0;
int16_t ticksCodeurDroitTab[Nmoy];
int16_t codeurDroitDeltaPos;
int16_t codeurDroitPos;

// Définitions et déclarations pour le codeur incrémental du moteur gauche
#define codeurGaucheInterruptionA 0
#define codeurGauchePinA 2
#define codeurGauchePinB 8
volatile long ticksCodeurGauche = 0;
uint8_t indiceTicksCodeurGauche = 0;
int16_t ticksCodeurGaucheTab[Nmoy];
int16_t codeurGaucheDeltaPos;
int16_t codeurGauchePos;

// Définitions et déclarations pour le capteur de distance
#define TRIGGER_PIN  11  // Envoi de l'impulsion ultrason
#define ECHO_PIN     13  // Réception de l'écho
#define MAX_DISTANCE 200 // Distance max en cm
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

// Certaines parties du programme sont exécutées à cadence fixe grâce à la bibliothèque FlexiTimer2.
// Cadence d'échantillonnage en ms
#define CADENCE_MS 10
volatile double dt = CADENCE_MS/1000.;

// Initialisations
void setup(void) {
  
  // Initialisation du bus I2C
  slave.init(20);

  // Codeur incrémental moteur droit
  pinMode(codeurDroitPinA, INPUT_PULLUP);      // entrée digitale pin A codeur
  pinMode(codeurDroitPinB, INPUT_PULLUP);      // entrée digitale pin B codeur
  // A chaque changement de niveau de tension sur le pin A du codeur,
  // on exécute la fonction GestionInterruptionCodeurDroitPinA (définie à la fin du programme)
  enableInterrupt(codeurDroitPinA, GestionInterruptionCodeurDroitPinA, CHANGE);
  
  // Codeur incrémental moteur gauche
  pinMode(codeurGauchePinA, INPUT_PULLUP);      // entrée digitale pin A codeur
  pinMode(codeurGauchePinB, INPUT_PULLUP);      // entrée digitale pin B codeur
  // A chaque changement de niveau de tension sur le pin A du codeur,
  // on exécute la fonction GestionInterruptionCodeurGauchePinA (définie à la fin du programme)
  enableInterrupt(codeurGauchePinA, GestionInterruptionCodeurGauchePinA, CHANGE);
  
  Serial.begin(115200);
//  while (!Serial) {
//    ; // Attente de la connexion du port série. Requis pour l'USB natif
//  }
    
  /* Le programme principal est constitué:
     - de la traditionnelle boucle "loop" des programmes Arduino, dans laquelle on lit 
       en permanence la liaison série pour récupérer les nouvelles consignes de mouvement
     - de la fonction "isrt", dont l'exécution est définie à cadence fixe par les deux instructions
       ci-dessous. La bonne méthode pour exécuter une fonction à cadence fixe, c'est de l'exécuter sur interruption
       (la boucle "loop" est alors interrompue régulièrement, à la cadence voulue, par un timer).
       Une mauvaise méthode serait d'utiliser la fonction Arduino "delay". En effet, la fonction "delay" définit
       un délai entre la fin des instructions qui la précèdent et le début des instructions qui la suivent, mais si
       on ne connait pas le temps d'exécution de ces instructions (ou si celui-ci est inconnu), on n'obtiendra
       jamais une exécution à cadence fixe.
       Il est nécessaire d'exécuter la fonction "isrt" à cadence fixe car cette fonction:
       - doit renvoyer des données à cadence fixe sur la liaison série, pour monitoring
       - calcule l'asservissement de verticalité et de mouvement, conçu pour fonctionner à une cadence bien spécifiée
     
  */
  FlexiTimer2::set(CADENCE_MS, 1/1000., isrt); // résolution timer = 1 ms
  FlexiTimer2::start();

}


uint16_t getCodeurDroitDeltaPos() {
  return (codeurDroitDeltaPos + 32768);
}

uint16_t getCodeurDroitPos() {
  return (codeurDroitPos + 32768);
}

void setCodeurDroitPos(int16_t value) {
  codeurDroitPos = value;
}

uint16_t getAnalogValue(uint8_t analogPin) {
  return analogRead(analogPin);
}

uint16_t getCodeurGaucheDeltaPos() {
  return (codeurGaucheDeltaPos + 32768);
}

uint16_t getCodeurGauchePos() {
  return (codeurGauchePos + 32768);
}

void setCodeurGauchePos(int16_t value) {
  codeurGauchePos = value;
}

uint16_t firmwareOK() {
  return 1;
}

uint16_t getDistance() {
  return sonar.ping_cm();
}

void checkCommands() {
  slave.checkCommand(1, getCodeurDroitDeltaPos);
  slave.checkCommand(2, getCodeurGaucheDeltaPos);
  slave.checkCommand(3, getCodeurDroitPos);
  slave.checkCommand(4, getCodeurGauchePos);
  slave.checkCommand(5, setCodeurDroitPos);
  slave.checkCommand(6, setCodeurGauchePos);
  slave.checkCommand(7, getAnalogValue);
  slave.checkCommand(8, firmwareOK);
  slave.checkCommand(9, getDistance);
}


// Boucle principale
void loop() {
  if(slave.commandReady()) {
    checkCommands();
    slave.commandDone();
  }
}

// Fonction excutée sur interruption
void isrt() {
  
  // Mesure de la vitesse des moteurs grâce aux codeurs incrémentaux
  
  // On calcule une moyenne glissante de la vitesse sur les Nmoy derniers échantillons
  // Nombre de ticks codeur depuis la dernière fois accumulé pour les 1à derniers échantillons
  // Ce nombre est mis à jour par les fonctions GestionInterruptionCodeurDroitPinA et GestionInterruptionCodeurDroitPinB,
  // exécutées à chaque interruption due à une impulsion sur la voie A ou B du codeur incrémental
  for (int i=0; i<Nmoy; i++) {
    ticksCodeurDroitTab[i] += ticksCodeurDroit;
  }  
  
  // Angle absolu
  codeurDroitPos += ticksCodeurDroit;
  
  // Une fois lu, ce nombre est remis à 0 pour pouvoir l'incrémenter de nouveau sans risquer de débordement de variable
  ticksCodeurDroit = 0;
  
  // Pour l'échantillon courant, calcule de l'angle de rotation du moteur pendant la période d'échantillonnage
  codeurDroitDeltaPos = ticksCodeurDroitTab[indiceTicksCodeurDroit];
  // Remise à zéro du compteur d'impulsion codeur pour l'échantillon courant
  ticksCodeurDroitTab[indiceTicksCodeurDroit] = 0;
  
  // Mise à jour de l'indice d'échantillon courant
  indiceTicksCodeurDroit++;
  if (indiceTicksCodeurDroit==Nmoy) {
    indiceTicksCodeurDroit = 0;
  }

  // On calcule une moyenne glissante de la vitesse sur les Nmoy derniers échantillons
  // Nombre de ticks codeur depuis la dernière fois accumulé pour les 1à derniers échantillons
  // Ce nombre est mis à jour par les fonctions GestionInterruptionCodeurGauchePinA et GestionInterruptionCodeurGauchePinB,
  // exécutées à chaque interruption due à une impulsion sur la voie A ou B du codeur incrémental
  for (int i=0; i<Nmoy; i++) {
    ticksCodeurGaucheTab[i] += ticksCodeurGauche;
  }  
  
  // Angle absolu
  codeurGauchePos += ticksCodeurGauche;
  
  // Une fois lu, ce nombre est remis à 0 pour pouvoir l'incrémenter de nouveau sans risquer de débordement de variable
  ticksCodeurGauche = 0;
  
  // Pour l'échantillon courant, calcule de l'angle de rotation du moteur pendant la période d'échantillonnage
  codeurGaucheDeltaPos = ticksCodeurGaucheTab[indiceTicksCodeurGauche];
  // Remise à zéro du compteur d'impulsion codeur pour l'échantillon courant
  ticksCodeurGaucheTab[indiceTicksCodeurGauche] = 0;
  
  // Mise à jour de l'indice d'échantillon courant
  indiceTicksCodeurGauche++;
  if (indiceTicksCodeurGauche==Nmoy) {
    indiceTicksCodeurGauche = 0;
  }
}


void GestionInterruptionCodeurDroitPinA() {  
  // Routine de service d'interruption attachée à la voie A du codeur incrémental droit
  // On utilise la fonction digitalReadFast2 de la librairie digitalWriteFast
  // car les lectures doivent être très rapides pour passer le moins de temps possible
  // dans cette fonction (appelée de nombreuses fois, à chaque impulsion de codeur)
  if (digitalReadFast2(codeurDroitPinA) == digitalReadFast2(codeurDroitPinB)) {
    ticksCodeurDroit++;
  }
  else {
    ticksCodeurDroit--;
  }
}


void GestionInterruptionCodeurGauchePinA() {
  // Routine de service d'interruption attachée à la voie A du codeur incrémental gauche
  // On utilise la fonction digitalReadFast2 de la librairie digitalWriteFast
  // car les lectures doivent être très rapides pour passer le moins de temps possible
  // dans cette fonction (appelée de nombreuses fois, à chaque impulsion de codeur)
  if (digitalReadFast2(codeurGauchePinA) == digitalReadFast2(codeurGauchePinB)) {
    ticksCodeurGauche++;
  }
  else {
    ticksCodeurGauche--;
  }
}

