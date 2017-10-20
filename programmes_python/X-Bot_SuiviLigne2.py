#!/usr/bin/python
# -*- coding: utf-8 -*-

##################################################################################
# Programme de suivi de ligne du robot X-Bot,
# disponible à l'adresse:
# http://boutique.3sigma.fr/12-robots
#
# Auteur: 3Sigma
# Version 1.0 - 01/10/2017
##################################################################################

# Importe les fonctions Arduino pour Python
from pyduino_pcduino import *

import time, sched
import os
import threading
import signal
import json
import sys

# Pour la détection d'adresse IP
import socket
import fcntl
import struct

# Pour le serveur de socket
import tornado.httpserver
import tornado.ioloop
from tornado.ioloop import PeriodicCallback
import tornado.web
import tornado.websocket
import tornado.template


# Nom de l'hostname (utilisé ensuite pour savoir sur quel système
# tourne ce programme)
hostname = socket.gethostname()

# Imports pour la communication i2c avec l'Iteaduino Uno
from uno import Uno
uno = Uno(hostname = hostname)

# Pour le calcul de la vitesse des moteurs
Nmoy = 10

codeurDroitDeltaPos = 0
codeurDroitDeltaPosPrec = 0
codeurGaucheDeltaPos = 0
codeurGaucheDeltaPosPrec = 0

directionMoteurDroit = 7
pwmMoteurDroit = 6

directionMoteurGauche = 4
pwmMoteurGauche = 5

tensionBatterie = 7.4


# Variables pour la commande en tension du moteur
vref = 0. # consigne de vitesse
vrefDroit = 0. # consigne vitesse de rotation du moteur droit
vrefGauche = 0. # consigne vitesse de rotation du moteur gauche
omegaDroit = 0. # vitesse de rotation du moteur droit
omegaGauche = 0. # vitesse de rotation du moteur gauche
commandeDroit = 0. # commande en tension pour le moteur droit
commandeGauche = 0. # commande en tension pour le moteur gauche
commande_avant_sat_Droit = 0. # valeur de la commande avant la saturation (voir ci-dessous) pour le moteur droit
commande_avant_sat_Gauche = 0. # valeur de la commande avant la saturation (voir ci-dessous) pour le moteur gauche
umax = 6. # valeur max de la tension de commande du moteur
umin = -6. # valeur min (ou max en négatif) de la tension de commande du moteur

# Déclarations pour les consignes de mouvement
vref = 0.

# Variables pour le suivi de ligne
L1 = 0.
L2 = 0.
L3 = 0.
seuil = 2.5

# Cadencement
T0 = time.time()
dt = 0.01
i = 0
tdebut = 0
# Création d'un scheduler pour exécuter des opérations à cadence fixe
s = sched.scheduler(time.time, time.sleep)

# Lecture de la tension d'alimentation
idecimLectureTension = 0
decimLectureTension = 6000
tensionAlim = 7.4
# Sécurité sur la tension d'alimentation
tensionAlimMin = 6.4;

#--- setup --- 
def setup():
    global tensionAlim
    pinMode(directionMoteurDroit, OUTPUT)
    pinMode(pwmMoteurDroit, OUTPUT)
    
    pinMode(directionMoteurGauche, OUTPUT)
    pinMode(pwmMoteurGauche, OUTPUT)
    
    CommandeMoteurDroit(0, tensionBatterie)
    CommandeMoteurGauche(0, tensionBatterie)
    
    # La mesure de la tension d'alimentation se fait via un pont diviseur de rapport 3 / 13
    # Les entrées analogiques A0 et A1 sont sur 6 bits avant une plage de variation de 2V
    # La résolution obtenue avec la commande suivante est donc très mauvaise
    #tensionAlimBrute = analogReadmV(A0)
    #tensionAlim = max(tensionAlimMin, float(tensionAlimBrute) * 13. / 3.) / 1000.
    # On préfère lire la tension à partir de la mesure faire par la carte Iteaduino Uno
    try:
        tensionAlimBrute = uno.analog_read(0)
        tensionAlimAvantMax = 3.3 * tensionAlimBrute * (13. / 3.) / 1024.;
        tensionAlim = max(tensionAlimMin, tensionAlimAvantMax);
        print "Tension d'alimentation", tensionAlim
    except:
        print "Probleme lecture tension d'alimentation"
        pass

    
# -- fin setup -- 
 
# -- loop -- 
def loop():
    global i
    i = i+1
    s.enterabs( T0 + (i * dt), 1, CalculVitesse, ())
    s.run()
# -- fin loop --


def CalculVitesse():
    global omegaDroit, omegaGauche, tdebut, codeurDroitDeltaPos, codeurGaucheDeltaPos, commandeDroit, commandeGauche, vref, \
        codeurDroitDeltaPosPrec, codeurGaucheDeltaPosPrec, vrefDroit, vrefGauche, \
        idecimLectureTension, decimLectureTension, tensionAlim, \
        L1, L2, L3, seuil
        
    # Mesure de la vitesse du moteur grâce aux codeurs incrémentaux
    try:        
        codeurDroitDeltaPos = uno.read_codeurDroitDeltaPos()
        
        codeurDroitDeltaPosPrec = codeurDroitDeltaPos
    except:
        #print "Erreur lecture codeur droit"
        codeurDroitDeltaPos = codeurDroitDeltaPosPrec
        pass
    
    try:        
        codeurGaucheDeltaPos = uno.read_codeurGaucheDeltaPos()
        
        codeurGaucheDeltaPosPrec = codeurGaucheDeltaPos
    except:
        #print "Erreur lecture codeur gauche"
        codeurGaucheDeltaPos = codeurGaucheDeltaPosPrec
        pass
    
    omegaDroit = -2 * ((2 * 3.141592 * codeurDroitDeltaPos) / 1632) / (Nmoy * dt)  # en rad/s
    omegaGauche = 2 * ((2 * 3.141592 * codeurGaucheDeltaPos) / 1632) / (Nmoy * dt)  # en rad/s
        
    # Lecture des capteurs
    try:
        L1 = 3.3 * uno.analog_read(1) / 1024.
    except:
        print "Probleme lecture capteur suivi de ligne 1"
        pass
        
    try:
        L2 = 3.3 * uno.analog_read(2) / 1024.
    except:
        print "Probleme lecture capteur suivi de ligne 2"
        pass
        
    try:
        L3 = 3.3 * uno.analog_read(3) / 1024.
    except:
        print "Probleme lecture capteur suivi de ligne 3"
        pass
        
    # mesuremV2 = analogReadmV(A2)
    # mesuremV3 = analogReadmV(A3)
    #print mesuremV1, mesuremV2, mesuremV3

    
    # On compare par rapport à un seuil pour savoir si le capteur voit la ligne ou non
    surLigne1 = False
    surLigne2 = False
    surLigne3 = False
    if L1 > seuil:
        surLigne1 = True
    if L2 > seuil:
        surLigne2 = True
    if L3 > seuil:
        surLigne3 = True
    
    # Si le robot est centré sur la ligne, on va tout droit
    if ((surLigne1 == False) and (surLigne2 == True) and (surLigne3 == False)) or ((surLigne1 == True) and (surLigne2 == True) and (surLigne3 == True)):
        vrefDroit = vref
        vrefGauche = vref
    # Si seul le capteur de droite est sur la ligne on tourne à droite fort
    elif (surLigne1 == True) and (surLigne2 == False) and (surLigne3 == False):
        vrefDroit = -vref * 2 / 3
        vrefGauche = vref
    # Si seul le capteur de gauche est sur la ligne on tourne à gauche fort
    elif (surLigne1 == False) and (surLigne2 == False) and (surLigne3 == True):
        vrefDroit = vref
        vrefGauche = -vref * 2 / 3
    # Si les deux capteurs de droite sont sur la ligne on tourne à droite normalement
    elif (surLigne1 == True) and (surLigne2 == True) and (surLigne3 == False):
        vrefDroit = -vref / 2
        vrefGauche = vref
    # Si les deux capteurs de gauche sont sur la ligne on tourne à gauche normalement
    elif (surLigne1 == False) and (surLigne2 == True) and (surLigne3 == True):
        vrefDroit = vref
        vrefGauche = -vref / 2

    # Calcul de la commande avant saturation
    commande_avant_sat_Droit = vrefDroit
    commande_avant_sat_Gauche = vrefGauche

    # Application de la saturation sur la commande
    if (commande_avant_sat_Droit > umax):
        commandeDroit = umax
    elif (commande_avant_sat_Droit < umin):
        commandeDroit = umin
    else:
        commandeDroit = commande_avant_sat_Droit
    
    if (commande_avant_sat_Gauche > umax) :
        commandeGauche = umax
    elif (commande_avant_sat_Gauche < umin):
        commandeGauche = umin
    else:
        commandeGauche = commande_avant_sat_Gauche


    CommandeMoteurDroit(commandeDroit, tensionBatterie)
    CommandeMoteurGauche(commandeGauche, tensionBatterie)
    
    # Lecture de la tension d'alimentation
    if idecimLectureTension >= decimLectureTension:
        # La mesure de la tension d'alimentation se fait via un pont diviseur de rapport 3 / 13
        # Les entrées analogiques A0 et A1 sont sur 6 bits avant une plage de variation de 2V
        # La résolution obtenue avec la commande suivante est donc très mauvaise
        #tensionAlimBrute = analogReadmV(A0)
        #tensionAlim = max(tensionAlimMin, float(tensionAlimBrute) * 13. / 3.) / 1000.
        # On préfère lire la tension à partir de la mesure faire par la carte Iteaduino Uno
        try:
            tensionAlimBrute = uno.analog_read(0)
            tensionAlimAvantMax = 3.3 * tensionAlimBrute * (13. / 3.) / 1024.;
            tensionAlim = max(tensionAlimMin, tensionAlimAvantMax);
            idecimLectureTension = 0
        except:
            print "Probleme lecture tension d'alimentation"
            pass
    else:
        idecimLectureTension = idecimLectureTension + 1
            
    

    
def CommandeMoteurDroit(tension, tensionAlim):
    # Cette fonction calcule et envoi les signaux PWM au pont en H
    # en fonction des tensions de commande et d'alimentation

    # Normalisation de la tension d'alimentation par
    # rapport à la tension d'alimentation
    tension_int = int(255 * tension / tensionAlim)

    # Saturation par sécurité
    if (tension_int > 255):
        tension_int = 255

    if (tension_int < -255):
        tension_int = -255

    # Commande PWM
    if (tension_int >= 0):
        digitalWrite(directionMoteurDroit, 0)
        analogWrite(pwmMoteurDroit, tension_int)

    if (tension_int < 0):
        digitalWrite(directionMoteurDroit, 1)
        analogWrite(pwmMoteurDroit, -tension_int)


    
def CommandeMoteurGauche(tension, tensionAlim):
    # Cette fonction calcule et envoi les signaux PWM au pont en H
    # en fonction des tensions de commande et d'alimentation

    # Normalisation de la tension d'alimentation par
    # rapport à la tension d'alimentation
    tension_int = int(255 * tension / tensionAlim)

    # Saturation par sécurité
    if (tension_int > 255):
        tension_int = 255

    if (tension_int < -255):
        tension_int = -255

    # Commande PWM
    if (tension_int >= 0):
        digitalWrite(directionMoteurGauche, 1)
        analogWrite(pwmMoteurGauche, tension_int)

    if (tension_int < 0):
        digitalWrite(directionMoteurGauche, 0)
        analogWrite(pwmMoteurGauche, -tension_int)



        
def emitData():
    # Délai nécessaire pour que le serveur ait le temps de démarrer
    delay(5000)
    while not noLoop: loop() # appelle fonction loop sans fin

    
class WSHandler(tornado.websocket.WebSocketHandler):
    def open(self):
        global socketOK
        print 'connection opened...'
        socketOK = True
        self.callback = PeriodicCallback(self.sendToSocket, 100)
        self.callback.start()
    

    def on_message(self, message):
        global vref, seuil

        jsonMessage = json.loads(message)
        
        if jsonMessage.get('vref') != None:
            vref = float(jsonMessage.get('vref'))
        if jsonMessage.get('seuil') != None:
            seuil = float(jsonMessage.get('seuil'))
          
        if not socketOK:
            vref = 0.


    def on_close(self):
        global socketOK, vrefDroit, vrefGauche
        print 'connection closed...'
        socketOK = False
        vrefDroit = 0.
        vrefGauche = 0.

    def sendToSocket(self):
        global vref, omegaGauche, omegaDroit, socketOK, commandeDroit, commandeGauche, tensionAlim, seuil, L1, L2, L3
        
        tcourant = time.time() - T0
        aEnvoyer = json.dumps({ 'Temps':("%.2f" % tcourant), \
                                'Consigne':("%.2f" % vref), \
                                'omegaDroit':("%.2f" % omegaDroit), \
                                'omegaGauche':("%.2f" % omegaGauche), \
                                'TensionAlim':("%.2f" % tensionAlim), \
                                'L1':("%.1f" % L1), \
                                'L2':("%.1f" % L2), \
                                'L3':("%.1f" % L3), \
                                'seuil_lu':("%.1f" % seuil), \
                                'Raw':("%.2f" % tcourant) + "," + \
                                ("%.2f" % vref) + "," + \
                                ("%.2f" % omegaDroit) + "," + \
                                ("%.2f" % omegaGauche) + "," + \
                                ("%.2f" % L1) + "," + \
                                ("%.2f" % L2) + "," + \
                                ("%.2f" % L3) + "," + \
                                ("%.2f" % tensionAlim)})
        if socketOK:
            try:
                self.write_message(aEnvoyer)
            except:
                pass
            
    def check_origin(self, origin):
        # Voir http://www.tornadoweb.org/en/stable/websocket.html#tornado.websocket.WebSocketHandler.check_origin
        # et http://www.arundhaj.com/blog/tornado-error-during-websocket-handshake.html
        return True        

    
def get_ip_address(ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915,  # SIOCGIFADDR
        struct.pack('256s', ifname[:15])
    )[20:24])
    
application = tornado.web.Application([
    (r'/ws', WSHandler)
])

def startTornado():
    http_server = tornado.httpserver.HTTPServer(application)
    http_server.listen(9090)
    tornado.ioloop.IOLoop.instance().start()


# Gestion du CTRL-C
def signal_handler(signal, frame):
    global vrefDroit, vrefGauche
    print 'You pressed Ctrl+C!'
    vrefDroit = 0.
    vrefGauche = 0.
    CommandeMoteurDroit(0, 5)
    CommandeMoteurGauche(0, 5)
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

#--- obligatoire pour lancement du code -- 
if __name__=="__main__": # pour rendre le code executable 

    # Test pour savoir si le firmware est présent sur la carte Uno
    firmwarePresent = False
    for i in range(1, 11):
        time.sleep(0.1)
        print "Test presence du firmware de la carte Uno, tentative " + str(i) + " / 10"
        try:
            firmwarePresent = uno.firmwareOK()
            if firmwarePresent:
                break
        except:
            print "Firmware absent"
            
    if firmwarePresent:
        print "Firmware present, on continue..."
        started = False
        startedDroit = False
        startedGauche = False
        setup() # appelle la fonction setup
        print "Setup done."
        
        th = threading.Thread(None, emitData, None, (), {})
        th.daemon = True
        th.start()
        
        print "Starting Tornado."
        try:
            print "Connect to ws://" + get_ip_address('eth0') + ":9090/ws with Ethernet."
        except:
            pass
            
        try:
            print "Connect to ws://" + get_ip_address('wlan0') + ":9090/ws with Wifi."
        except:
            pass
        socketOK = False
        startTornado()
    else:
        print "Firmware absent, on abandonne ce programme."
        print "Veuillez charger le firmware sur la carte Uno pour exécuter ce programme."


