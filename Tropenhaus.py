#--------------------------------------------------------------------------------------------------#
# Sensordatenüberwachung "Oldenburg Animals, Tropenhaus, Schmetterlingshaus"
# Sven Wille, ETS2021, Version 2.2 vom 18.06.2022

# Folgende Informationen stehen in der README:  Verweis zur Projektdokumentation
#                                               Hyperlinks zu den Bibliotheken

# Die Temperatur soll über 25°C und unter 30°C betragen. 
# Die Luftfeuchtigkeit soll Über 70% und unter 90% liegen. 
# Der ESP32 soll sich automatisch mit den eingestellten WLAN SSID verbinden 
# Der Wasserstand soll über dem Analogwert 500 liegen
# Die Messwerte der Sensoren sollen über das JSON Datenformat und über das Mqtt-Protokoll
# an den Broker Mosquitto und Node-RED als Ganzzahl gesendet werden.

# Hardware: HTU2X --> I2C  STL = Pin 22, SDA = Pin 21  Temperatur und Luftfeuchtigkeit
#           Matrix max7219 --> SPI  sck = Pin 15, mosi = Pin 2, cs = Pin 17 / 4x8x8 Matrix
#           SE045 Wasserstandsensor --> ADC (analog)

# Bibliotheken: HTU2X.py, max7219.py
#--------------------------------------------------------------------------------------------------#

#----------------------------------Initalisierung--------------------------------------------------#

from machine import Pin, SPI, SoftI2C, ADC      # Pin (LEDs), SPI (LED Matrix), SoftI2C (HTU2X), ADC(analog Pin)
import max7219                                  # LED Matrix importieren
import network                                  # Netzwerkanbindung
from umqtt.simple import MQTTClient             # MQTT Protokoll (Datenformatübertragung)
from HTU2X import HTU21D                        # HTU21D (Tem, Luftfeuchte)
from time import sleep                          # Sleep aus Bibliothek time
import json                                     # Datenformat Json importieren

#----------------------------------Netzwerkverbindung & MQTT---------------------------------------#

MQTT_SERVER= "192.168.178.34"
CLIENT_ID= "Wille"
MQTT_TOPIC = "Tropenhaus/Sensordaten"

SSID = "FRITZ!Box 7590 JI"
PASSWORT ="170911200695071094"
#SSID = "BZTG-IoT"
#PASSWORT= "WerderBremen24"

wlan = network.WLAN(network.STA_IF)     # Objekt wlan als interface
wlan.active(True)                       # System einschalten
if not wlan.isconnected():              
    wlan.connect(SSID, PASSWORT)        # wlan verbinden, wenn nicht verbunden
    while not wlan.isconnected():       #solange nicht verbunden ist mache nichts
        pass
    print("Netzwerkkonfiguration: ", wlan.ifconfig()) # Ausgabe Netzwerkkonfiguration

#---------------------------Initalisierung Hardware----------------------------------------------#
# Analog-Pin für Füllstandsensor
analog_Pin = ADC(Pin(33))           # Analog Pin 33 
analog_Pin.atten(ADC.ATTN_11DB)     # 3.3V Spannungsbereich 

# Temperatur und Luftfeuchtigkeitssensor 
htux_sensor = HTU21D(22,21)  

# LEDs für Grenzwertüberwachung
led_Temp_Green = Pin(13,Pin.OUT)
led_Temp_Red = Pin(32,Pin.OUT)
led_water_empty_red = Pin(27,Pin.OUT)
led_Water_low_yellow = Pin(26,Pin.OUT)
led_water_good_green = Pin(25,Pin.OUT)

# Led-Matrix 4 mal 8x8 Display
spi = SPI(1, baudrate = 1000000, polarity = 1, phase = 0, sck = Pin(15), mosi = Pin(2))  # mosi = DIN; sck = CLK
ss = Pin(17, Pin.OUT)                                                                    # ss = cs  
matrix = max7219.Matrix8x8(spi, ss, 4) 
matrix.brightness(2)

#------------------- Funktion: Wasserstand zu niedrig---------------------------------------------------#
def alarm():
    for i in range (-1,9):                      # Schleifenausgabe Water!
        matrix.fill(0)                          # Display Ausgabe löschen
        matrix.text("Water!",-i,0,1)            # Ausgabe String, rechts, höhe
        matrix.show()                           # Text Anzeigen
        sleep(0.25)                             # Schlafen 0,25s und dann neu

#-------------- Funktion: Ausgabe der Temperatur & Luftfeuchtigkeit auf der Matrix ---------------------#
def ausgabe_Anzeige_Temp_Luft():
    matrix.fill(0)
    matrix.text((ausgabe_temperatur+" C"),0,0,1)
    matrix.fill_rect(21,0,2,2,1)
    matrix.show()
    sleep(2)
    matrix.fill(0)
    matrix.text((ausgabe_luftfeuchte+"%"),0,0,1)
    matrix.show()
    sleep(2)

#-------- Funktion: Entsprechende LED's für Grenzwerte Temperatur & Luftfeuchtigkeit ------------------#
def grenzwerte_Temp_Luft():
    if temperatur < 25 or temperatur > 30 or luftfeuchte < 70 or luftfeuchte >90: 
        led_Temp_Red.value(1)
        led_Temp_Green.value(0)
    else: 
        led_Temp_Red.value(0)
        led_Temp_Green.value(1)

#---- Funktion: Definition des Wasserstandes (LEDs ansprechen und entsprechende Funktion ausühren) ----#
def wasserstand():
    if water_value < 500:
        alarm()                                        # Funktion Alarm auf Matrix ausführen
        led_water_empty_red.value(1)
        led_water_good_green.value(0)
        led_Water_low_yellow.value(0) 
    else:
        ausgabe_Anzeige_Temp_Luft()                    # Funktion Temp & Luftfeuchte auf Matrix ausführen
        if water_value > 500 and water_value < 700:
            led_water_empty_red.value(0)
            led_water_good_green.value(0)
            led_Water_low_yellow.value(1)
        else:
            led_water_empty_red.value(0)
            led_water_good_green.value(1)
            led_Water_low_yellow.value(0)

# -------------------------------- Dauerschleife ------------------------------------------------------ # 
while True: 

    mqttwille = MQTTClient(CLIENT_ID, MQTT_SERVER)      # Objekt aus MQTT ID und IP Adresse Broker
    mqttwille.connect()                                 # MQTT Wille verbinden

    messwerte_temp = []                                 # Liste für Temperaturmesswerte instanzieren
    messwerte_luftfeuchte = []                          # Liste für Luftfeuchtigkeitsmesswerte instanzieren

    for i in range (0,15):                              # 15 Messwerte jeweils in die entsprechende Liste schreiben
        temperatur = round(htux_sensor.temperature)
        luftfeuchte = round(htux_sensor.humidity)
        messwerte_temp.append(temperatur)
        messwerte_luftfeuchte.append(luftfeuchte)

    messwerte_temp.sort()                               # sortieren der Messwerte
    messwerte_luftfeuchte.sort()
    del messwerte_temp[0]                               # ersten Listeneintrag löschen
    del messwerte_luftfeuchte[0]
    messwerte_temp.pop()                                # letzten Listeneintrag löschen
    messwerte_luftfeuchte.pop()

# Durchschnitswerte berechen und in Ausgabe schreiben
    messwerte_Durchschnitt_Temp = round(sum(messwerte_temp)/ len(messwerte_temp))                      
    messwerte_Durchschnitt_Luftfeuchte = round(sum(messwerte_luftfeuchte)/len(messwerte_luftfeuchte))
    ausgabe_temperatur = str(messwerte_Durchschnitt_Temp)
    ausgabe_luftfeuchte = str(messwerte_Durchschnitt_Luftfeuchte)
  
    del messwerte_temp[:]           # löschen der Listen
    del messwerte_luftfeuchte[:]

    grenzwerte_Temp_Luft()          # Funktion LED Grenzwerte Temp & Luftfeuchte ausführen
   
    water_value = analog_Pin.read()         # Analogwert für Wasserstand auslesen
    ausgabe_fuellstand = str(water_value)

    # Ausgabe der Messwerte auf dem Terminal
    print("Wasserstand: {0}" .format(water_value))                
    print("Temperatur: {0} Grad Celcius" .format(messwerte_Durchschnitt_Temp))
    print("Luftfeuchtigkeit: {0} Prozent" .format(messwerte_Durchschnitt_Luftfeuchte))

    wasserstand()                   # Funktion Wasserstand ausführen

#-----------------------------------------JSON Datenformat---------------------------------------------#
    daten_Sensorwerte = {
        "Daten" :[
            {   
                "Temperatur": ausgabe_temperatur,
                "Luftfeuchte": ausgabe_luftfeuchte,
                "Fuellstand": ausgabe_fuellstand
            }
        ]    
    }

#----------------- Publishen der Messwerte zum Broker Moaquitto -> NodeRED ----------------------------#
    mqttwille.publish(MQTT_TOPIC,json.dumps(daten_Sensorwerte))

    sleep(1)
    mqttwille.disconnect()
