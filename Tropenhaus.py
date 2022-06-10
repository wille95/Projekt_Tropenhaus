import max7219                                  # Display Matrix
import network 
from umqtt.simple import MQTTClient              
from machine import Pin, SPI, SoftI2C, ADC      # Pin EIn-/Ausgäng, Busssystem Display Matrix, Bussystem Sensor, ADC(analog Pin)
from HTU2X import HTU21D
from time import sleep
import json

MQTT_SERVER= "192.168.1.179"
CLIENT_ID= "Wille"
MQTT_TOPIC = "Tropenhaus/Sensordaten"
SSID = "BZTG-IoT"
PASSWORT ="WerderBremen24"

# Analog-Pin Füllstandsensor
analog_Pin = ADC(Pin(33))           # Analog Pin 33 
analog_Pin.atten(ADC.ATTN_11DB)     # 3.3V Spannungsbereich 

# Temperatur und Luftfeuchtigkeitssensor 
htux_sensor = HTU21D(22,21)   

wlan = network.WLAN(network.STA_IF)     #objekt wlan als interface
wlan.active(True)                       #system einschalten
if not wlan.isconnected():              #wenn wlan nicht verbunden ist
    wlan.connect(SSID, PASSWORT)  #wlan verbinden
    while not wlan.isconnected():               #solange nicht verbunden ist mache nichts
        pass
    print("Netzwerkkonfiguration: ", wlan.ifconfig())

# Matrix 8x8 Display
spi = SPI(1, baudrate = 1000000, polarity = 1, phase = 0, sck = Pin(32), mosi = Pin(27))  # mosi = DIN; sck = CLK
ss = Pin(33, Pin.OUT)                                                                     # ss = cs  
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


while True: 

    mqttwille = MQTTClient(CLIENT_ID, MQTT_SERVER)
    mqttwille.connect()
    print("MQTT_verbunden!")

    temperatur = round(htux_sensor.temperature)
    luftfeuchte = round(htux_sensor.humidity)
    ausgabe_luftfeuchte = str(luftfeuchte)
    ausgabe_temperatur = str(temperatur)

    water_value = analog_Pin.read()   # Analogwert auslesen
    print(water_value)                # Analogwer ausgeben

    if temperatur > 30:
        alarm() 
    else:
        ausgabe_Anzeige_Temp_Luft() 
    
    daten_Sensorwerte = {
        "Daten" :[
            {   
                "Temperatur": ausgabe_temperatur,
                "Luftfeuchte": ausgabe_luftfeuchte
            }
        ]    
    }

    mqttwille.publish(MQTT_TOPIC,json.dumps(daten_Sensorwerte))

    sleep(1)
    mqttwille.disconnect()
    print("MQTT verbunden!")

    