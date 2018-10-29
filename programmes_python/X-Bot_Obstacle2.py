#!/usr/bin/python
# -*- coding: utf-8 -*-

##################################################################################
# Pilotage du robot X-Bot,
# disponible à l'adresse:
# http://boutique.3sigma.fr/12-robots
#
# Auteur: 3Sigma
# Version 1.2 - 29/10/2018
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

omega = 0.

# Déclarations pour le capteur de distance
idecimDistance = 0
distance = 0


# Les moteurs sont asservis en vitesse grâce à un régulateur de type PID
# On déclare ci-dessous les variables et paramètres nécessaires à l'asservissement et au régulateur
R = 0.045 # Rayon d'une roue
W = 0.14 # Largeur du robot
umax = 6. # valeur max de la tension de commande du moteur
umin = -6. # valeur min (ou max en négatif) de la tension de commande du moteur
vxmes = 0. # vitesse longitudinale mesurée
ximes = 0. # vitesse de rotation mesurée
Tf = 0.02 # constante de temps de filtrage de l'action dérivée du PID
Kpvx = 2.5 # gain proportionnel pour l'asservissement de vitesse longitudinale
Kivx = 50 # gain intégral pour l'asservissement de vitesse longitudinale
Kdvx = 0.00 # gain dérivé pour l'asservissement de vitesse longitudinale
Kpxi = 0.25 # gain proportionnel pour l'asservissement de rotation
Kixi = 5 # gain intégral pour l'asservissement de rotation
Kdxi = 0.000 # gain dérivé pour l'asservissement de rotation
commande_avant_sat_vx = 0. # commande avant la saturation pour l'asservissement de vitesse longitudinale
commande_vx = 0. # commande pour l'asservissement de vitesse longitudinale
commande_avant_sat_xi = 0. # commande avant la saturation pour l'asservissement de rotation
commande_xi = 0. # commande pour l'asservissement de rotation
commandeDroit = 0. # commande en tension calculée par le PID pour le moteur droit
commandeGauche = 0. # commande en tension calculée par le PID pour le moteur gauche
omegaDroit = 0. # Vitesse moteur droit
omegaGauche = 0. # Vitesse moteur gauche

I_x = [0., 0.]
D_x = [0., 0.]
erreurprec = [0., 0.] # mesure de la vitesse du moteur droit au calcul précédent

# Déclarations pour les consignes de mouvement
vxref = 0.
xiref = 0.


# Cadencement
T0 = time.time()
dt = 0.01
i = 0
tdebut = 0
tprec = time.time()
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
    global started, omegaDroit, omegaGauche, tdebut, tprec, \
        codeurDroitDeltaPos, codeurGaucheDeltaPos, commandeDroit, commandeGauche, vxmes, ximes, vxref, xiref, \
        distance, idecimDistance, idecimLectureTension, decimLectureTension, tensionAlim
            
    tdebut = time.time()

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

    # Lecture de la distance mesurée par le capteur ultrason
    if idecimDistance >= 10:
        idecimDistance = 0
        try:
            distance = uno.read_distance()
            if distance == 0:
                # Correspond en fait à une distance supérieure à 200 cm
                distance = 200
            print "Distance: ", distance, " cm"
        except:
            print "Probleme lecture distance"
            pass
    else:
        idecimDistance = idecimDistance + 1


    # Application de la consigne lue
    if (tdebut - T0 > 10) and  (distance > 0):    
        if (distance < 40):
            vxref = 0.
            xiref = 3.14
        else:
            xiref = 0

    # Définition des entrées de la fonction d'asservissement
    vxmes = (omegaDroit + omegaGauche)*R/2
    ximes = -(omegaDroit - omegaGauche)*R/W

    dt2 = time.time() - tprec
    tprec = time.time()
    
    # Asservissement PID
    commande_vx = PID(0, vxref, vxmes, Kpvx, Kivx, Kdvx, Tf, umax, umin, dt2);
    commande_xi = PID(1, xiref, ximes, Kpxi, Kixi, Kdxi, Tf, umax, umin, dt2);
    
    # Calcul des commandes des moteurs
    commandeDroit = (commande_vx - commande_xi);
    commandeGauche = (commande_vx + commande_xi);
      
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

    #print(time.time() - tdebut)

    
def PID(iMoteur, ref, mes, Kp, Ki, Kd, Tf, umax, umin, dt2):
    global I_x, D_x, erreurprec
    
    # Calcul du PID
    # Paramètres intermédiaires
    br = 1. / (Kp + 0.0001)
    ad = Tf / (Tf + dt2);
    bd = Kd / (Tf + dt2);

    # Calcul de la commande avant saturation
    erreur = ref - mes
    
    # Terme proportionnel
    P_x = Kp * erreur

    # Terme dérivé
    D_x[iMoteur] = ad * D_x[iMoteur] + bd * (erreur - erreurprec[iMoteur])

    # Calcul de la commande avant saturation
    if (Ki == 0):
        I_x[iMoteur] = 0.
    commande_avant_sat = P_x + I_x[iMoteur] + D_x[iMoteur]

    # Application de la saturation sur la commande
    if (commande_avant_sat > umax):
        commande = umax
    elif (commande_avant_sat < umin):
        commande = umin
    else:
        commande = commande_avant_sat
    
    # Terme intégral (sera utilisé lors du pas d'échantillonnage suivant)
    I_x[iMoteur] = I_x[iMoteur] + Ki * dt2 * (erreur + br * (commande - commande_avant_sat))
    
    # Stockage de la mesure courante pour utilisation lors du pas d'échantillonnage suivant
    erreurprec[iMoteur] = erreur
    
    return commande

        
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
    global tprec
    # Délai nécessaire pour que le serveur ait le temps de démarrer
    #delay(5000)
    tprec = time.time()
    while not noLoop: loop() # appelle fonction loop sans fin

    
class WSHandler(tornado.websocket.WebSocketHandler):
    def open(self):
        global socketOK
        print 'connection opened...'
        socketOK = True
        self.callback = PeriodicCallback(self.sendToSocket, 100)
        self.callback.start()
    
    def on_message(self, message):
        global vxref

        jsonMessage = json.loads(message)
        
        if jsonMessage.get('vxref') != None:
            vxref = float(jsonMessage.get('vxref'))
          
        if not socketOK:
            vxref = 0.

 

    def on_close(self):
        global socketOK, commandeDroit, commandeGauche
        print 'connection closed...'
        socketOK = False
        commandeDroit = 0.
        commandeGauche = 0.

    def sendToSocket(self):
        global started, codeurDroitDeltaPos, codeurGaucheDeltaPos, socketOK, commandeDroit, commandeGauche, vxref, xiref, vxmes, ximes, \
            omegaDroit, omegaGauche, vxref
        
        tcourant = time.time() - T0
        aEnvoyer = json.dumps({ 'Temps':("%.2f" % tcourant),
                                'Consigne vitesse longitudinale':("%.2f" % vxref), \
                                'Vitesse longitudinale':("%.2f" % vxmes),
                                'Vitesse de rotation':("%.2f" % (180 * ximes/3.141592)),
                                'omegaDroit':("%.2f" % omegaDroit),
                                'omegaGauche':("%.2f" % omegaGauche),
                                'commandeDroit':("%.2f" % commandeDroit),
                                'commandeGauche':("%.2f" % commandeGauche),
                                'TensionAlim':("%.2f" % tensionAlim), \
                                'distance':("%.2f" % distance), \
                                'Raw':("%.2f" % tcourant) + "," +
                                ("%.2f" % vxmes) + "," +
                                ("%.2f" % (180 * ximes/3.141592)) + "," +
                                ("%.2f" % omegaDroit) + "," +
                                ("%.2f" % omegaGauche) + "," +
                                ("%.2f" % commandeDroit) + "," +
                                ("%.2f" % commandeGauche) + "," +
                                ("%.2f" % tensionAlim) + "," +
                                ("%.2f" % distance)})
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
    global commandeDroit, commandeGauche
    print 'You pressed Ctrl+C!'
    commandeDroit = 0.
    commandeGauche = 0.
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


