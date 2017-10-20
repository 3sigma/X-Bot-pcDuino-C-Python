/******************************************************************************
Programmation de la carte Iteaduino Uno du robot X-Bot à base de pcDuino,
disponible à l'adresse:
http://boutique.3sigma.fr/12-robots

La définition des consignes de vitesse se fait dans le sous-programme "isrt":
- vxref permet de définir la consigne de vitesse longitudinale (en m/s)
  vxref doit rester inférieur à 0.5 m/s
- xiref permet de définir la consigne de vitesse de rotation (rad/s) autour de l'axe vertical
  xiref doit rester inférieur à 3 rad/s

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

// Définitions et déclarations pour le codeur incrémental du moteur droit
#define codeurDroitPinA 3
#define codeurDroitPinB 9
#define Nmoy 10
volatile long ticksCodeurDroit = 0;
int indiceTicksCodeurDroit = 0;
int ticksCodeurDroitTab[Nmoy];
int codeurDroitDeltaPos;

// Définitions et déclarations pour le codeur incrémental du moteur gauche
#define codeurGauchePinA 2
#define codeurGauchePinB 8
volatile long ticksCodeurGauche = 0;
int indiceTicksCodeurGauche = 0;
int ticksCodeurGaucheTab[Nmoy];
int codeurGaucheDeltaPos;

// Définitions et déclarations pour le moteur à courant continu.
#define directionMoteurDroit  7
#define pwmMoteurDroit  6
#define directionMoteurGauche  4
#define pwmMoteurGauche  5

// Tension de référence du système
float Vref = 3.3;

// Tension d'alimentation mesurée
#define pinTensionAlim  A0
int tensionAlimBrute = 1023;
float tensionAlim = 7.4;
// Sécurité sur la tension d'alimentation
float tensionAlimAvantMax;
float tensionAlimMin = 6.4;



// Certaines parties du programme sont exécutées à cadence fixe grâce à la bibliothèque FlexiTimer2.
// Cadence d'échantillonnage en ms
#define CADENCE_MS 10
volatile double dt = CADENCE_MS/1000.;
volatile double temps = -CADENCE_MS/1000.;

// Le gyropode est asservi grâce à plusieurs régulateurs de type PI
// On déclare ci-dessous les variables et paramètres nécessaires à l'asservissement et au régulateur
double R = 0.045; // Rayon d'une roue
double W = 0.14; // Largeur du gyropode
double umax = 6.; // valeur max de la tension de commande du moteur
double umin = -6.; // valeur min (ou max en négatif) de la tension de commande du moteur
double vxmes = 0.; // vitesse longitudinale mesurée
double ximes = 0.; // vitesse de rotation mesurée
double Kpvx = 1; // gain proportionnel pour l'asservissement de vitesse longitudinale
double Kivx = 10; // gain intégral pour l'asservissement de vitesse longitudinale
double Kpxi = 0.1; // gain proportionnel pour l'asservissement de rotation
double Kixi = 1.; // gain intégral pour l'asservissement de rotation
double commande_vx = 0.; // commande pour l'asservissement de vitesse longitudinale
double commande_xi = 0.; // commande pour l'asservissement de rotation
double P_vx = 0.; // action proportionnelle pour l'asservissement de vitesse longitudinale
double I_vx = 0.; // action intégrale pour l'asservissement de vitesse longitudinale
double P_xi = 0.; // action proportionnelle pour l'asservissement de rotation
double I_xi = 0.; // action intégrale pour l'asservissement de rotation

// Déclarations pour la commande des moteurs
double omegaDroit = 0.;
double omegaGauche = 0.;
double commandeDroit = 0.;
double commandeGauche = 0.;

// Déclarations pour les consignes de mouvement
double vxref = 0.;
double xiref = 0.;
double Tmax = 0.;


// Initialisations
void setup(void) {
  // Il est nécessaire d'attendre au moins 30 secondes pour que le pcDuino ait démarré
  // et ne perturbe pas l'Arduino
  delay(30000);
  
  // Codeur incrémental moteur droit
  pinMode(codeurDroitPinA, INPUT_PULLUP);      // entrée digitale pin A codeur
  pinMode(codeurDroitPinB, INPUT_PULLUP);      // entrée digitale pin B codeur
  // A chaque changement de niveau de tension sur le pin A du codeur,
  // on exécute la fonction GestionInterruptionCodeurDroitPinA (définie à la fin du programme)
  enableInterrupt(codeurDroitPinA, GestionInterruptionCodeurDroitPinA, CHANGE);

  // Codeur incrémental moteur droit
  pinMode(codeurGauchePinA, INPUT_PULLUP);      // entrée digitale pin A codeur
  pinMode(codeurGauchePinB, INPUT_PULLUP);      // entrée digitale pin B codeur
  // A chaque changement de niveau de tension sur le pin A du codeur,
  // on exécute la fonction GestionInterruptionCodeurGauchePinA (définie à la fin du programme)
  enableInterrupt(codeurGauchePinA, GestionInterruptionCodeurGauchePinA, CHANGE);

  // Moteur à courant continu droit
  pinMode(directionMoteurDroit, OUTPUT);
  pinMode(pwmMoteurDroit, OUTPUT);

  // Moteur à courant continu gauche
  pinMode(directionMoteurGauche, OUTPUT);
  pinMode(pwmMoteurGauche, OUTPUT);

  // Liaison série
  // ATTENTION: ne pas modifier la vitesse de transmission de la liaison série,
  // sinon le programme de pilotage exécuté sur le PC connecté à la carte ne fonctionnera plus
  Serial.begin(115200);
  Serial.flush();
  
  // Lecture de la tension d'alimentation via un pont diviseur de tension de 3/13
  tensionAlimBrute = analogRead(pinTensionAlim);
  tensionAlimAvantMax = Vref * ((float)tensionAlimBrute) * (13. / 3.) / 1024.;
  tensionAlim = max(tensionAlimMin, tensionAlimAvantMax);
  Serial.print("Tension alim: ");
  Serial.println(tensionAlim);

  // Compteur d'impulsions des encodeurs
  ticksCodeurDroit = 0;
  ticksCodeurGauche = 0;
        
  
  /* Le programme principal est constitué:
     - de la traditionnelle boucle "loop" des programmes Arduino
     - de la fonction "isrt", dont l'exécution est définie à cadence fixe par les deux instructions
       ci-dessous. La bonne méthode pour exécuter une fonction à cadence fixe, c'est de l'exécuter sur interruption
       (la boucle "loop" est alors interrompue régulièrement, à la cadence voulue, par un timer).
       Une mauvaise méthode serait d'utiliser la fonction Arduino "delay". En effet, la fonction "delay" définit
       un délai entre la fin des instructions qui la précèdent et le début des instructions qui la suivent, mais si
       on ne connait pas le temps d'exécution de ces instructions (ou si celui-ci est inconnu), on n'obtiendra
       jamais une exécution à cadence fixe.
       Il est nécessaire d'exécuter la fonction "isrt" à cadence fixe car cette fonction
       calcule l'asservissement de verticalité et de mouvement, conçu pour fonctionner à une cadence bien spécifiée
     
  */
  FlexiTimer2::set(CADENCE_MS, 1/1000., isrt); // résolution timer = 1 ms
  FlexiTimer2::start();

}

// Boucle principale
void loop() {
}

// Fonction excutée sur interruption
void isrt(){

  // Compteur de boucle
  int i;

  // Mesure de la vitesse des moteurs grâce aux codeurs incrémentaux
  
  // On calcule une moyenne glissante de la vitesse sur les Nmoy derniers échantillons
  // Nombre de ticks codeur depuis la dernière fois accumulé pour les 1à derniers échantillons
  // Ce nombre est mis à jour par les fonctions GestionInterruptionCodeurDroitPinA et GestionInterruptionCodeurDroitPinA,
  // exécutées à chaque interruption due à une impulsion sur la voie A ou B du codeur incrémental
  for (int i=0; i<Nmoy; i++) {
    ticksCodeurDroitTab[i] += ticksCodeurDroit;
  }  
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
  // Ce nombre est mis à jour par les fonctions GestionInterruptionCodeurGauchePinA et GestionInterruptionCodeurGauchePinA,
  // exécutées à chaque interruption due à une impulsion sur la voie A ou B du codeur incrémental
  for (int i=0; i<Nmoy; i++) {
    ticksCodeurGaucheTab[i] += ticksCodeurGauche;
  }  
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

  // Calcul de la vitesse de rotation. C'est le nombre d'impulsions converti en radian, divisé par la période d'échantillonnage
  // On fait un calcul glissant sur Nmoy échantillons, d'où le Nmoy*dt
  // ATTENTION: le facteur 2 devant est là car avec 2 moteurs, la résolution est divisée par 2
  omegaDroit = -2*((2.*3.141592*((double)codeurDroitDeltaPos))/1632.)/(Nmoy*dt);  // en rad/s
  omegaGauche = 2*((2.*3.141592*((double)codeurGaucheDeltaPos))/1632.)/(Nmoy*dt);  // en rad/s

  // Consignes
  // - vxref permet de définir la consigne de vitesse longitudinale (en m/s)
  //   vxref doit rester inférieur à 0.5 m/s
  // - xiref permet de définir la consigne de vitesse de rotation (rad/s) autour de l'axe vertical
  //   xiref doit rester inférieur à 3 rad/s
  // Ci-dessous, le robot décrit un 8
  if (temps > Tmax) {
    Tmax = temps + 6.; // Le 8 dure 6 s
  }
  else if (temps < (Tmax-5.5)) { // La ligne droite dure 0.5 s
    vxref = 0.3;
    xiref = 0.;
  }
  else if (temps < (Tmax-3.)) { // Le virage dure 2.5 s
    vxref = 0.3;
    xiref = 2.;
  }
  else if (temps < (Tmax-2.5)) { // La ligne droite dure 0.5 s
    vxref = 0.3;
    xiref = 0.;
  }
  else { // // Le virage dure 2.5 s
    vxref = 0.3;
    xiref = -2.;
  }

//  vxref = 0.3;
//  xiref = -2.;
  
  // Définition des entrées de la fonction d'asservissement
  vxmes = (omegaDroit + omegaGauche)*R/2;
  ximes = (omegaDroit - omegaGauche)*R/W;

  /******* Calcul du PI sur vx *******/
  
  // Terme proportionnel
  P_vx = Kpvx * (vxref - vxmes);
  
  // Calcul de la commande avant saturation
  commande_vx = P_vx + I_vx;
    
  // Terme intégral (sera utilisé lors du pas d'échantillonnage suivant)
  I_vx = I_vx + Kivx * dt * (vxref - vxmes);

  /******* Fin Calcul du PI sur vx *******/
  
  /******* Calcul du PI sur xi *******/
  // Terme proportionnel
  P_xi = Kpxi * (xiref - ximes);
  
  // Calcul de la commande avant saturation
  commande_xi = P_xi + I_xi;
  
  // Terme intégral (sera utilisé lors du pas d'échantillonnage suivant)
  I_xi = I_xi + Kixi * dt * (xiref - ximes);

  /******* Fin Calcul du PI sur xi *******/

  
  // Calcul des commandes des moteurs
  commandeDroit = (commande_vx + commande_xi);
  commandeGauche = (commande_vx - commande_xi);
  
  // Application des commandes aux moteurs
  CommandeMoteurDroit(commandeDroit, tensionAlim);
  CommandeMoteurGauche(commandeGauche, tensionAlim);
  
  // Incrémentation du temps courant
  temps += dt;
}

void CommandeMoteurDroit(float tension, float tensionAlim) {
  // Cette fonction calcule et envoi les signaux PWM au pont en H
  // en fonction des tensions de commande et d'alimentation
  int tension_int;
  
  // Normalisation de la tension d'alimentation par
  // rapport à la tension d'alimentation
  tension_int = (int)(255*(tension/tensionAlim));
  
  // Saturation par sécurité
  if (tension_int>255) {
    tension_int = 255;
  }
  if (tension_int<-255) {
    tension_int = -255;
  }
  
  // Commande PWM
  if (tension_int>=0) {
    digitalWrite(directionMoteurDroit, LOW);
    analogWrite(pwmMoteurDroit, tension_int);
  }
  if (tension_int<0) {
    digitalWrite(directionMoteurDroit, HIGH);
    analogWrite(pwmMoteurDroit, -tension_int);
  }
}

void CommandeMoteurGauche(float tension, float tensionAlim) {
  // Cette fonction calcule et envoi les signaux PWM au pont en H
  // en fonction des tensions de commande et d'alimentation
  int tension_int;
  
  // Normalisation de la tension d'alimentation par
  // rapport à la tension d'alimentation
  tension_int = (int)(255*(tension/tensionAlim));
  
  // Saturation par sécurité
  if (tension_int>255) {
    tension_int = 255;
  }
  if (tension_int<-255) {
    tension_int = -255;
  }
  
  // Commande PWM
  if (tension_int>=0) {
    digitalWrite(directionMoteurGauche, HIGH);
    analogWrite(pwmMoteurGauche, tension_int);
  }
  if (tension_int<0) {
    digitalWrite(directionMoteurGauche, LOW);
    analogWrite(pwmMoteurGauche, -tension_int);
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

