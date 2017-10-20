#!/usr/bin/python
# -*- coding: utf-8 -*-

##################################################################################
# Programme de commande en tension des moteurs du robot Geeros ou X-Bot,
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

omega = 0.

# Les moteurs sont asservis en vitesse grâce à un régulateur de type PID
# On déclare ci-dessous les variables et paramètres nécessaires à l'asservissement et au régulateur
vref = 0. # consigne de vitesse
vrefDroit = 0. # consigne vitesse de rotation du moteur droit
vrefGauche = 0. # consigne vitesse de rotation du moteur gauche
omegaDroit = 0. # vitesse de rotation du moteur droit
omegaGauche = 0. # vitesse de rotation du moteur gauche
commandeDroit = 0. # commande en tension calculée par le PID pour le moteur droit
commandeGauche = 0. # commande en tension calculée par le PID pour le moteur gauche
commande_avant_sat_Droit = 0. # valeur de la commande avant la saturation (voir ci-dessous) pour le moteur droit
commande_avant_sat_Gauche = 0. # valeur de la commande avant la saturation (voir ci-dessous) pour le moteur gauche
umax = 6. # valeur max de la tension de commande du moteur
umin = -6. # valeur min (ou max en négatif) de la tension de commande du moteur
yprecDroit = 0. # mesure de la vitesse du moteur droit au calcul précédent
yprecGauche = 0. # mesure de la vitesse du moteur gauche au calcul précédent
Tf = 0.02 # constante de temps de filtrage de l'action dérivée du PID
P_x_Droit = 0. # valeur de l'action proportionnelle sur le moteur droit
I_x_Droit = 0. # valeur de l'action intégrale sur le moteur droit
D_x_Droit = 0. # valeur de l'action dérivée sur le moteur droit
P_x_Gauche = 0. # valeur de l'action proportionnelle sur le moteur gauche
I_x_Gauche = 0. # valeur de l'action intégrale sur le moteur gauche
D_x_Gauche = 0. # valeur de l'action dérivée sur le moteur gauche
# Variables intermédiaires
Ti = 0.
Td = 0.
Tt = 0.
ad = 0.
bd = 0.
br = 0.


# Variables utilisées pour les données reçues
typeSignal = 0
offset = 0.
amplitude = 0.
frequence = 0.
Kp = 0.25 # gain proportionnel du PID
Ki = 5.0 # gain intégral du PID
Kd = 0.000 # gain dérivé du PID
moteurint = 0

# Cadencement
T0 = time.time()
dt = 0.01
i = 0
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
    global omegaDroit, omegaGauche, timeLastReceived, timedOut, commandeDroit, commandeGauche, vrefDroit, vrefGauche, vref, \
        P_x_Droit, I_x_Droit, D_x_Droit, P_x_Gauche, I_x_Gauche, D_x_Gauche, yprecDroit, yprecGauche, dt2, tprec, \
        typeSignal, offset, amplitude, frequence, Kp, Ki, Kd, moteurint, \
        Ti, Td, Tt, ad, bd, br, \
        idecimLectureTension, decimLectureTension, tensionAlim
                
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
    
    tcourant = time.time() - T0
    # Calcul de la consigne en fonction des données reçues
    if typeSignal == 0: # signal carré
        if frequence > 0:
            if (tcourant - (float(int(tcourant*frequence)))/frequence < 1/(2*frequence)):
                vref = offset + amplitude
            else:
                vref = offset
        else:
            vref = offset + amplitude
    else: # sinus
        if frequence > 0:
            vref = offset + amplitude * sin(2*3.141592*frequence*tcourant)
        else:
            vref = offset + amplitude

    # Application de la consigne sur chaque moteur
    if moteurint == 0:
        vrefDroit = vref
        vrefGauche = 0.
    elif moteurint == 1:
        vrefDroit = 0.
        vrefGauche = vref
    elif moteurint == 2:
        vrefDroit = vref
        vrefGauche = vref
    else:
        vrefDroit = 0.
        vrefGauche = 0.
            

    dt2 = time.time() - tprec
    tprec = time.time()
    
    # Calcul du PID
    # Paramètres intermédiaires
    Ti = Ki/(Kp+0.01)
    if (Kd>0): # Si PID
        ad = Tf/(Tf+dt2)
        bd = Kd/(Tf+dt2)
        Td = Kp/Kd
        Tt = sqrt(Ti*Td)
    else: # Si PI
        ad = 0.
        bd = 0.
        Tt = 0.5*Ti
    
    br = dt2/(Tt+0.01)

    # Terme proportionnel
    P_x_Droit = Kp * (vrefDroit - omegaDroit)
    P_x_Gauche = Kp * (vrefGauche - omegaGauche)

    # Terme dérivé
    D_x_Droit = ad * D_x_Droit - bd * (omegaDroit - yprecDroit)
    D_x_Gauche = ad * D_x_Gauche - bd * (omegaGauche - yprecGauche)

    # Calcul de la commande avant saturation
    commande_avant_sat_Droit = P_x_Droit + I_x_Droit + D_x_Droit
    commande_avant_sat_Gauche = P_x_Gauche + I_x_Gauche + D_x_Gauche

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

    # Terme intégral (sera utilisé lors du pas d'échantillonnage suivant)
    I_x_Droit = I_x_Droit + Ki * dt2 * (vrefDroit - omegaDroit) + br * (commandeDroit - commande_avant_sat_Droit)
    I_x_Gauche = I_x_Gauche + Ki * dt2 * (vrefGauche - omegaGauche) + br * (commandeGauche - commande_avant_sat_Gauche)
    
    # Stockage de la mesure courante pour utilisation lors du pas d'échantillonnage suivant
    yprecDroit = omegaDroit
    yprecGauche = omegaGauche

    # Fin Calcul du PID
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
    global tprec
    # Délai nécessaire pour que le serveur ait le temps de démarrer
    delay(5000)
    tprec = time.time()
    while not noLoop: loop() # appelle fonction loop sans fin

    
class WSHandler(tornado.websocket.WebSocketHandler):
    def open(self):
        global socketOK
        print 'connection opened...'
        socketOK = True
        self.callback = PeriodicCallback(self.sendToSocket, 20)
        self.callback.start()
    

    def on_message(self, message):
        global vref, vrefDroit, vrefGauche, typeSignal, offset, amplitude, frequence, Kp, Ki, Kd, moteurint, timeLastReceived, timedOut
        jsonMessage = json.loads(message)
        
        # Annulation du timeout de réception des données
        timeLastReceived = time.time()
        timedOut = False;
        
        if jsonMessage.get('typeSignal') != None:
            typeSignal = int(jsonMessage.get('typeSignal'))
        if jsonMessage.get('offset') != None:
            offset = float(jsonMessage.get('offset'))
        if jsonMessage.get('amplitude') != None:
            amplitude = float(jsonMessage.get('amplitude'))
        if jsonMessage.get('frequence') != None:
            frequence = float(jsonMessage.get('frequence'))
        if jsonMessage.get('Kp') != None:
            Kp = float(jsonMessage.get('Kp'))
        if jsonMessage.get('Ki') != None:
            Ki = float(jsonMessage.get('Ki'))
        if jsonMessage.get('Kd') != None:
            Kd = float(jsonMessage.get('Kd'))
        if jsonMessage.get('moteurint') != None:
            moteurint = int(jsonMessage.get('moteurint'))
        
        if not socketOK:
            typeSignal = 0
            offset = 0.
            amplitude = 0.
            frequence = 0.
  

    def on_close(self):
        global socketOK, vrefDroit, vrefGauche
        print 'connection closed...'
        socketOK = False
        vrefDroit = 0.
        vrefGauche = 0.

    def sendToSocket(self):
        global socketOK, vref, omegaDroit, omegaGauche, commandeDroit, commandeGauche, tensionAlim
        
        tcourant = time.time() - T0
        aEnvoyer = json.dumps({ 'Temps':("%.2f" % tcourant), \
                                'Consigne':("%.2f" % vref), \
                                'omegaDroit':("%.2f" % omegaDroit), \
                                'omegaGauche':("%.2f" % omegaGauche), \
                                'commandeDroit':("%.2f" % commandeDroit), \
                                'commandeGauche':("%.2f" % commandeGauche), \
                                'tensionAlim':("%.2f" % tensionAlim), \
                                'Raw':("%.2f" % tcourant) + "," +  \
                                ("%.2f" % vref) + "," +  \
                                ("%.2f" % omegaDroit) + "," +  \
                                ("%.2f" % omegaGauche) + "," +  \
                                ("%.2f" % commandeDroit) + "," +  \
                                ("%.2f" % commandeGauche) + "," + \
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


