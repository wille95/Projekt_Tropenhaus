#--------------------------------#
#Aufgabenbeschreibung#
#--------------------------------#

import max7219                                  # Display Matrix
import network 
from umqtt.simple import MQTTClient              
from machine import Pin, SPI, SoftI2C, ADC      # Pin EIn-/Ausgäng, Busssystem Display Matrix, Bussystem Sensor, ADC(analog Pin)
from HTU2X import HTU21D
from time import sleep
import json

MQTT_SERVER= "192.168.178.21"
CLIENT_ID= "Wille"
MQTT_TOPIC = "Tropenhaus/Sensordaten"
SSID = "FRITZ!Box 7590 JI"
PASSWORT ="170911200695071094"

wlan = network.WLAN(network.STA_IF)     #objekt wlan als interface
wlan.active(True)                       #system einschalten
if not wlan.isconnected():              #wenn wlan nicht verbunden ist
    wlan.connect(SSID, PASSWORT)  #wlan verbinden
    while not wlan.isconnected():               #solange nicht verbunden ist mache nichts
        pass
    print("Netzwerkkonfiguration: ", wlan.ifconfig())

# Analog-Pin Füllstandsensor
analog_Pin = ADC(Pin(33))           # Analog Pin 33 
analog_Pin.atten(ADC.ATTN_11DB)     # 3.3V Spannungsbereich 

# Temperatur und Luftfeuchtigkeitssensor 
htux_sensor = HTU21D(22,21)  

# LEDs
led_Temp_Green = Pin(13,Pin.OUT)
led_Temp_Red = Pin(32,Pin.OUT)
led_water_empty_red = Pin(27,Pin.OUT)
led_Water_low_yellow = Pin(26,Pin.OUT)
led_water_good_green = Pin(25,Pin.OUT)

# Matrix 8x8 Display
spi = SPI(1, baudrate = 1000000, polarity = 1, phase = 0, sck = Pin(15), mosi = Pin(2))  # mosi = DIN; sck = CLK
ss = Pin(17, Pin.OUT)                                                                     # ss = cs  
matrix = max7219.Matrix8x8(spi, ss, 4) 
matrix.fill(0)
matrix.brightness(2)

def alarm():
    matrix.fill(0)
    matrix.text(("STOP"),0,0,1)
    matrix.show()

def ausgabe_Anzeige_Temp_Luft():
    matrix.fill(0)
    matrix.text((ausgabe_temperatur+" C"),0,0,1)
    matrix.fill_rect(21,0,2,2,1)
    matrix.show()
    sleep(3)
    matrix.fill(0)
    matrix.text((ausgabe_luftfeuchte+"%"),0,0,1)
    matrix.show()
    sleep(3)            # time anstatt sleep!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

def grenzwerte_Temp_Luft():
    if temperatur < 25 or luftfeuchte < 60: 
        led_Temp_Red.value(1)
        led_Temp_Green.value(0)
    else: 
        led_Temp_Red.value(0)
        led_Temp_Green.value(1)

def wasserstand():
    if water_value < 500:
        alarm()
        led_water_empty_red.value(1)
        led_water_good_green.value(0)
        led_Water_low_yellow.value(0) 
    else:
        ausgabe_Anzeige_Temp_Luft()
        if water_value > 500 and water_value < 700:
            led_water_empty_red.value(0)
            led_water_good_green.value(0)
            led_Water_low_yellow.value(1)
        else:
            led_water_empty_red.value(0)
            led_water_good_green.value(1)
            led_Water_low_yellow.value(0)

while True: 

    mqttwille = MQTTClient(CLIENT_ID, MQTT_SERVER)
    mqttwille.connect()

    temperatur = round(htux_sensor.temperature)
    luftfeuchte = round(htux_sensor.humidity)
    ausgabe_luftfeuchte = str(luftfeuchte)
    ausgabe_temperatur = str(temperatur)

    grenzwerte_Temp_Luft()
   
    water_value = analog_Pin.read()   # Analogwert auslesen
    print(water_value)                # Analogwer ausgeben
    print(temperatur)
    print(luftfeuchte)

    wasserstand()

    daten_Sensorwerte = {
        "Daten" :[
            {   
                "Temperatur": ausgabe_temperatur,
                "Luftfeuchte": ausgabe_luftfeuchte
            }
        ]    
    }

    mqttwille.publish(MQTT_TOPIC,json.dumps(daten_Sensorwerte))

    sleep(0.5)
    mqttwille.disconnect()

    