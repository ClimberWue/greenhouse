#!/usr/bin/python
# -*- coding:utf-8 -*-
import paho.mqtt.client as mqtt

import os
import io
import glob
import time
import datetime
import sys
from ctypes import c_short
from ctypes import c_byte
from ctypes import c_ubyte

import tkinter
from tkinter import *
from tkinter import ttk

import board
import busio
import smbus
import adafruit_bmp280
from adafruit_bme280 import basic as adafruit_bme280              #has nearly a temperature shift of 2 degree Celsius
#import adafruit_bmp085
#import adafruit_bmp180
import adafruit_bmp3xx
from digitalio import DigitalInOut, Direction
import adafruit_bh1750
import adafruit_ccs811
import adafruit_sht31d
from adafruit_htu21d import HTU21D

import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
#derzeit sind noch andere ads1x15 Bibliotheken installiert -
#referenziert wird die adafruit_circuitpython_ads1x15 ohne Großbuchstaben im Namen!

import math, struct, array, fcntl
import RPi.GPIO as GPIO

#Einrichten von autostart
#--> https://techgeeks.de/raspberry-pi-autostart-von-programmen/
#--> www.netzmafia.de/skripten/hardware/RaspPi/RasPi_Auto.html
#--> http://christian-brauweiler.de/autostart-unter-raspian/  So läuft es
# über /home/pi/.config/lxsession/LXDE-pi/autostart

#configuration
winOut = True
shellOut = True
MQTTout = True
useBMP280 = False
useBME280 = False
useBME280ext = False
useBMP3xx = True
useBMP3xx_SPI = False
useLightSensor = True
useHumiditySensor = True
useSoilHumiditySensor = True
useCO2Sensor = True
useCCS811Sensor = True

#seconds for measurement cycle
waitSeconds = 10#60 sec. or more

#initialize relais IO
relais_1_GPIO =  16           #O2 fres air
relais_2_GPIO =  6            #air cooling start temperature
GPIO.setmode(GPIO.BCM)
GPIO.setup(relais_1_GPIO, GPIO.OUT)
GPIO.setup(relais_2_GPIO, GPIO.OUT)
GPIO.output(relais_1_GPIO, GPIO.LOW)
GPIO.output(relais_2_GPIO, GPIO.LOW)

# fresh air == O2 intake parameters - in Teilen von 1.0!
O2concentration = 0.205    # nachzuführen bei nachlassender Sensor Leistung --> kleiner werdend
# Ausgangswert war wohl 0.21 zufälligerweise
O2minforFreshAir = 0.195   # dto. mit Differenz von 1-2% absolutem O² Gehalt
O2hysteresis = 0.1  #0.05        # dto. etwa die Hälfte der Differenz beider obiger O² Werte

relais_1_status = 1# Achtung konträre Logik!!!
ventFreshAir = False
GPIO.output(relais_1_GPIO, relais_1_status)

# maximaler CO2 Wert sollte dauerhaft 2.000 ppm nicht überschreiten?
# bis 5.000ppm =0,5% darf man noch arbeiten
# ab 40.000ppm wäre gesundheitsgefährdend (=Atemluft 4%) bis tödlich ~8%!!!
# der aktuelle Lüfter mit 150mm bietet etwa 250 m³/h Luftstrom abzüglich Leitungsverluste
# das GWH hat bei 10 m² Grundfläche und 2-2,5 m Höhe etwa 22 m³ Rauminhalt.
# eine Minute Lüften saugt 3(-4) m³ Innenluft ab.

# secondary inhouse fan parameters
airTemp = 20.0
airTempCooling = 20    #test only was 32.0
relais_2_status = 1
GPIO.output(relais_2_GPIO, relais_2_status)

DEVICE = 0x76  # Standard-Geräteaddresse am I2C
BUS = smbus.SMBus(1)

def get_short(data, index):
    return c_short((data[index+1] << 8) + data[index]).value

def get_ushort(data, index):
    return (data[index+1] << 8) + data[index]

def get_char(data, index):
    result = data[index]
    if result > 127:
        result -= 256
    return result

def get_uchar(data, index):
    result = data[index] & 0xFF
    return result

def read_bme280id(addr=DEVICE):
    reg_id = 0xD0
    (chip_id, chip_version) = BUS.read_i2c_block_data(addr, reg_id, 2)
    return chip_id, chip_version

def read_bme280_all(addr=DEVICE):
    reg_data = 0xF7
    reg_control = 0xF4
    reg_config = 0xF5
    reg_control_hum = 0xF2
    reg_hum_msb = 0xFD
    reg_hum_lsb = 0xFE
    oversample_temp = 2
    oversample_pres = 2
    mode = 1
    oversample_hum = 2
    BUS.write_byte_data(addr, reg_control_hum, oversample_hum)
    control = oversample_temp << 5 | oversample_pres << 2 | mode
    BUS.write_byte_data(addr, reg_control, control)
    cal1 = BUS.read_i2c_block_data(addr, 0x88, 24)
    cal2 = BUS.read_i2c_block_data(addr, 0xA1, 1)
    cal3 = BUS.read_i2c_block_data(addr, 0xE1, 7)
    dig_t1 = get_ushort(cal1, 0)
    dig_t2 = get_short(cal1, 2)
    dig_t3 = get_short(cal1, 4)
    dig_p1 = get_ushort(cal1, 6)
    dig_p2 = get_short(cal1, 8)
    dig_p3 = get_short(cal1, 10)
    dig_p4 = get_short(cal1, 12)
    dig_p5 = get_short(cal1, 14)
    dig_p6 = get_short(cal1, 16)
    dig_p7 = get_short(cal1, 18)
    dig_p8 = get_short(cal1, 20)
    dig_p9 = get_short(cal1, 22)
    dig_h1 = get_uchar(cal2, 0)
    dig_h2 = get_short(cal3, 0)
    dig_h3 = get_uchar(cal3, 2)
    dig_h4 = get_char(cal3, 3)
    dig_h4 = (dig_h4 << 24) >> 20
    dig_h4 = dig_h4 | (get_char(cal3, 4) & 0x0F)
    dig_h5 = get_char(cal3, 5)
    dig_h5 = (dig_h5 << 24) >> 20
    dig_h5 = dig_h5 | (get_uchar(cal3, 4) >> 4 & 0x0F)
    dig_h6 = get_char(cal3, 6)
    wait_time = 1.25 + (2.3 * oversample_temp) + ((2.3 * oversample_pres) + 0.575) + ((2.3 * oversample_hum)+0.575)
    time.sleep(wait_time/1000)
    data = BUS.read_i2c_block_data(addr, reg_data, 8)
    pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
    temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
    hum_raw = (data[6] << 8) | data[7]
    var1 = ((((temp_raw>>3)-(dig_t1<<1)))*(dig_t2)) >> 11
    var2 = (((((temp_raw >> 4) - dig_t1) * ((temp_raw >> 4) - dig_t1)) >> 12) * dig_t3) >> 14
    t_fine = var1+var2
    temperature = float(((t_fine * 5) + 128) >> 8)
    var1 = t_fine / 2.0 - 64000.0
    var2 = var1 * var1 * dig_p6 / 32768.0
    var2 = var2 + var1 * dig_p5 * 2.0
    var2 = var2 / 4.0 + dig_p4 * 65536.0
    var1 = (dig_p3 * var1 * var1 / 524288.0 + dig_p2 * var1) / 524288.0
    var1 = (1.0 + var1 / 32768.0) * dig_p1
    if var1 == 0:
        pressure = 0
    else:
        pressure = 1048576.0 - pres_raw
        pressure = ((pressure - var2 / 4096.0) * 6250.0) / var1
        var1 = dig_p9 * pressure * pressure / 2147483648.0
        var2 = pressure * dig_p8 / 32768.0
        pressure = pressure + (var1 + var2 + dig_p7) / 16.0
    humidity = t_fine - 76800.0
    humidity = (hum_raw - (dig_h4 * 64.0 + dig_h5 / 16384.0 * humidity)) * \
               (dig_h2 / 65536.0 * (1.0 + dig_h6 / 67108864.0 * humidity * (1.0 + dig_h3 / 67108864.0 * humidity)))
    humidity = humidity * (1.0 - dig_h1 * humidity / 524288.0)
    if humidity > 100:
        humidity = 100
    elif humidity < 0:
        humidity = 0
    return temperature/100.0, pressure/100.0, humidity


#grove ADC
from grove.adc import ADC

class GroveAirQualitySensor(object):
    '''
    Grove Air Quality Sensor class

    Args:
        pin(int): number of analog pin/channel the sensor connected.
    '''
    def __init__(self, channel):
        self.channel = channel
        self.adc = ADC()

    @property
    def value(self):
        '''
        Get the air quality strength value, badest value is 100.0%.

        Returns:
            (int): ratio, 0(0.0%) - 1000(100.0%)
        '''
        return self.adc.read(self.channel)

class GroveO2Sensor(object):
    def __init__(self, channel):
        self.channel = channel
        self.adc = ADC()

    @property
    def value(self):
        return self.adc.read(self.channel)

class GroveSoilHumiditySensor(object):
    def __init__(self, channel):
        self.channel = channel
        self.adc = ADC()

    @property
    def value(self):
        return self.adc.read(self.channel)


#CO2 sensor T6703
bus = 1
addressT6703 = 0x15
I2C_SLAVE = 0x0703

class i2c_(object):
    def __init__(self, device, bus):

        self.fr = io.open("/dev/i2c-"+str(bus), "rb", buffering=0)
        self.fw = io.open("/dev/i2c-"+str(bus), "wb", buffering=0)

        # set device address
        fcntl.ioctl(self.fr, I2C_SLAVE, device)
        fcntl.ioctl(self.fw, I2C_SLAVE, device)

    def write(self, bytes):
        self.fw.write(bytes)

    def read(self, bytes):
        return self.fr.read(bytes)

    def close(self):
        self.fw.close()
        self.fr.close()

class T6703(object):
    def __init__(self):
        self.dev = i2c_(addressT6703, bus)

    def status(self):
        buffer = array.array('B', [0x04, 0x13, 0x8a, 0x00, 0x01])
        self.dev.write(buffer)
        time.sleep(0.1)
        data = self.dev.read(4)
        buffer = array.array('B', data)
        return buffer[2]*256+buffer[3]

    def gasPPM(self):
        buffer = array.array('B', [0x04, 0x13, 0x8b, 0x00, 0x01])
        self.dev.write(buffer)
        time.sleep(0.1)
        data = self.dev.read(4)
        buffer = array.array('B', data)
        return buffer[2]*256+buffer[3]

    def checkABC(self):
        buffer = array.array('B', [0x04, 0x03, 0xee, 0x00, 0x01])
        self.dev.write(buffer)
        time.sleep(0.1)
        data = self.dev.read(4)
        buffer = array.array('B', data)
        return buffer[2]*256+buffer[3]

    def calibrate(self):
        buffer = array.array('B', [0x05, 0x03, 0xec, 0xff, 0x00])
        self.dev.write(buffer)
        time.sleep(0.1)
        data = self.dev.read(5)
        buffer = array.array('B', data)
        return buffer[3]*256+buffer[3]


def shellPrint(st):
    print(st)
    
def debugPrint(st):
    print(st, file=sys.stderr)
    
def exceptionPrint(exc, st):
    print(exc, st, file=sys.stderr)
    
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        debugPrint("connected OK")

#MQTT endpoint        
broker = "GWHpi4"
client = mqtt.Client("GWH")
client.on_connect = on_connect

#client.connect("Rotauge", 1883, 60)
client.connect(broker)
client.loop_start()
time.sleep(4)
#SPI setup
spi = board.SPI() #board.SCLK, board.MOSI, board.MISO)
cs = DigitalInOut(board.CE1) #??? was D5
#bmp_SPI = adafruit_bmp3xx.BMP3XX_SPI(spi, cs)

# Treiberinstallation
os.system('sudo modprobe w1-gpio')
#os.system('sudo modprobe w1-therm')   # gives error                                                                                                                                                                                                                                                                                                                                                                                                                                                             os.system('modprobe w1-therm')

# Sensor als Objekt erstellen in Abhaengigkeit von I2C
i2c = busio.I2C(board.SCL, board.SDA)
#time.sleep(2)

#if useBMP3xx_SPI == True:
    #sensorBMP3xx_SPI = adafruit_bmp3xx.BMP3XX_SPI(spi, cs=0)

if useLightSensor == True:
    lightSensor = adafruit_bh1750.BH1750(i2c, address = 0x23)
#if useLightSensor == 0:
#    debugPrint("BH1750 failed!")

if useHumiditySensor == True:
    humiditySensor = HTU21D(i2c)
    humiditymin = 100.0
    humiditymax = 0.0

if useSoilHumiditySensor == True:
    soilHumiditySensor = adafruit_sht31d.SHT31D(i2c)
    soilHumiditymin = 100.0
    soilHumiditymax = 0.0

if useBMP280 == True:
    sensorBMP280 = adafruit_bmp280.Adafruit_BMP280_I2C(i2c, address=0x76)
if useBME280 == True:
    sensorBME280 = adafruit_bme280.Adafruit_BME280_I2C(i2c, address=0x76)

if useBMP3xx == True:
    sensorBMP3xx = adafruit_bmp3xx.BMP3XX_I2C(i2c, address=0x77)


ads = ADS.ADS1115(i2c, address=0x49)
# Create single-ended input on channel 0
chanSoilHum = AnalogIn(ads, ADS.P3)      #humSoil
# Create single-ended input on channel 3
chanGas = AnalogIn(ads, ADS.P1)      #Gas sensor
chan2 = AnalogIn(ads, ADS.P2)
chanO2 = AnalogIn(ads, ADS.P0)      #O2 Sensor

if useCO2Sensor == True:
    CO2sensor = T6703()    # min read with this sensor 372ppm
    CO2min = 10000
    CO2max = 0
    
if useCCS811Sensor == True:
    try:
        ccs811 = adafruit_ccs811.CCS811(i2c)
        while not ccs811.data_ready:
            pass
        temp = ccs811.temperature
        ccs811.temp_offset = temp - 25.0
    except Exception as exc:
        exceptionPrint(exc, "CCS811 Exception Start")

#grove ADC
from grove.helper import SlotHelper

#SoilHumiditySensor2 = GroveSoilHumiditySensor(4)
#SoilHumiditySensor = GroveSoilHumiditySensor(5)

#O2sensor = GroveO2Sensor(0)
#do it wituout grove
O2min = 30.0
O2max = 0.0

#Netzmafia BMP280
if useBMP280 == True:
    sensorBMP280.sea_level_pressure = 1013.25
    ALTITUDE = 210.0
if useBME280 == True:
    sensorBME280.sea_level_pressure = 1013.25
    ALTITUDE = 210.0
if useBME280ext == True:
    #sensorBME280.sea_level_pressure = 1013.25
    ALTITUDE = 210.0
if useBMP3xx == True:
    sensorBMP3xx.sea_level_pressure = 1013.25
    ALTITUDE = 210.0

#28-051692fef6ff #3m
#28-03168b2cf0ff #3m
#28-00000887cdd4 #Transistor
#28-0517a199f4ff #
#28-0417a228a7ff #
#28-00000ab5fa1a #
#28-3c01d60702cf #1.2m

proto = False

# Temperatursensor initialisieren 
base_dir = '/sys/bus/w1/devices/'
def read_rom(device_folder):
    name_file=device_folder+'/name'
    f = open(name_file,'r')
    return f.readline()

# Sensor Temperatur lesen lassen
def read_temp_raw(filedev):
    f = open(filedev, 'r')
    lines = f.readlines()
    f.close()
    if proto == True:
        print('lines: ', lines)
    return lines

# Temperatur auslesen
def read_temp(filename):
    lines = read_temp_raw(filename)
    while len(lines) < 2 or lines[0].strip()[-3:] != 'YES':
        #hier könnte eine schärfere Überprüfung formuliert werden,
        #z.B. Rückgabe eines NAN Werts nach 10 Durchgängen
        time.sleep(0.2)
        lines = read_temp_raw(filename)
    equals_pos = lines[1].find('t=')
    if equals_pos != -1:
        temp_string = lines[1][equals_pos+2:]
        temp_c = float(temp_string) / 1000.0
        return temp_c
    return 85.0      #Überlaufwert NAN des Chips; bei Kontaktproblemen auftretend

startTime = datetime.datetime.now()
if winOut == True:
    root = Tk()
    root.title("Gewächshaus")

    tW = Text(root, width = 50, height = 32)#, wrap = "none")
    tW.pack()
    #mainloop() # not appropriate
    #quellen --> https://stackoverflow.com/questions/54237067/how-to-make-tkinter-gui-thread-safe

winLine = 0

def winPrint(tkWin, st):
    global winLine
    tkWin.insert(tkinter.END, st)
    tkWin.insert(tkinter.END, "\n")
    winLine += 1

def winClear(tkWin):
    global winLine
    winLine = 0
    tkWin.delete('1.0', tkinter.END)


# Ausgabe
while True:
    if winOut == True:
        winClear(tW)
        
    dt = datetime.datetime.now()
    if winOut == True:
        winPrint(tW, dt)
    if shellOut == True:
        shellPrint(dt)
    
    # interne Systemtemperatur
    try:
        fsys = open("/sys/class/thermal/thermal_zone0/temp", "r")
        cpu_temp = fsys.readline()
        fsys.close()
        ct = float(cpu_temp)
    except:
        ct = 0.0
    st1 = "Temperatur CPU: {0:0.1f} °C".format((ct / 1000))
    if winOut == True:
        winPrint(tW, st1)
    if shellOut == True:
        shellPrint(st1)
    if MQTTout:
        client.publish("gwh/tempCPU", "{0:0.1f}".format(ct / 1000))
    
    # Ausgabe der Messwerte BMP280
    if useBMP280 == True:
        try:
            pressure = sensorBMP280.pressure
            pressure_nn = pressure/pow(1 - ALTITUDE/44330.0, 5.255)
            airTemp = temperature = sensorBMP280.temperature
            altitude = sensorBMP280.altitude
            st1 = "Temperatur:  {0:0.1f} °C".format(temperature)
            st2 = "Luftdruck:   {0:0.1f} hPa".format(pressure)
            st3 = "LuftdruckNN: {0:0.1f} hPa".format(pressure_nn)
            st4 = "Höhe:  {0:0.2f} m".format(altitude)
            if winOut == True:
                winPrint(tW, st1)
                winPrint(tW, st2)
                winPrint(tW, st3)
                winPrint(tW, st4)
            if shellOut == True:
                shellPrint(st1)
                shellPrint(st2)
                shellPrint(st3)
                shellPrint(st4)
            if MQTTout:
                client.publish("gwh/tempBoard", "{0:0.1f}".format(temperature))
                client.publish("gwh/pressure", "{0:0.1f}".format(pressure))
                client.publish("gwh/pressureNN", "{0:0.1f}".format(pressure_nn))
                client.publish("gwh/altitude", "{0:0.2f}".format(altitude))
        except Exception as exc:
            exceptionPrint(exc, "BMP280 Exception")
            
    # Ausgabe der Messwerte BMP280
    if useBME280 == True:
        try:
            pressure = sensorBME280.pressure
            pressure_nn = pressure/pow(1 - ALTITUDE/44330.0, 5.255)
            airTemp = temperature = sensorBME280.temperature
            altitude = sensorBME280.altitude
            st1 = "Temperatur:  {0:0.1f} °C".format(temperature)
            st2 = "Luftdruck:   {0:0.1f} hPa".format(pressure)
            st3 = "LuftdruckNN: {0:0.1f} hPa".format(pressure_nn)
            st4 = "Höhe:  {0:0.2f} m".format(altitude)
            if winOut == True:
                winPrint(tW, st1)
                winPrint(tW, st2)
                winPrint(tW, st3)
                winPrint(tW, st4)
            if shellOut == True:
                shellPrint(st1)
                shellPrint(st2)
                shellPrint(st3)
                shellPrint(st4)
            if MQTTout:
                client.publish("gwh/tempBoard", "{0:0.1f}".format(temperature))
                client.publish("gwh/pressure", "{0:0.1f}".format(pressure))
                client.publish("gwh/pressureNN", "{0:0.1f}".format(pressure_nn))
                client.publish("gwh/altitude", "{0:0.2f}".format(altitude))
        except Exception as exc:
            exceptionPrint(exc, "BME280 Exception")

     # Ausgabe der Messwerte BMP280
    if useBME280ext == True:
        try:
            temperature, pressure, humidity = read_bme280_all()
            #pressure_nn = pressure/pow(1 - ALTITUDE/44330.0, 5.255)
            airTemp = temperature
            st1 = "Temperatur:  {0:0.1f} °C".format(temperature)
            st2 = "Luftdruck:   {0:0.1f} hPa".format(pressure)
            if winOut == True:
                winPrint(tW, st1)
                winPrint(tW, st2)
            if shellOut == True:
                shellPrint(st1)
                shellPrint(st2)
            if MQTTout:
                client.publish("gwh/tempBoard", "{0:0.1f}".format(temperature))
                client.publish("gwh/pressure", "{0:0.1f}".format(pressure))
        except Exception as exc:
            exceptionPrint(exc, "BME280ext Exception")

    if useBMP3xx == True:
        try:
            pressure3 = sensorBMP3xx.pressure
            pressure_nn3 = pressure3/pow(1 - ALTITUDE/44330.0, 5.255)
            airTemp = temperature3 = sensorBMP3xx.temperature
            altitude3 = sensorBMP3xx.altitude
            st1 = "Temperatur 3:  {0:0.1f} °C".format(temperature3)
            st2 = "Luftdruck 3:   {0:0.2f} hPa".format(pressure3)
            st3 = "LuftdruckNN 3: {0:0.2f} hPa".format(pressure_nn3)
            st4 = "Höhe 3:        {0:0.2f} m".format(altitude3)
            if winOut == True:
                winPrint(tW, st1)
                winPrint(tW, st2)
                winPrint(tW, st3)
                winPrint(tW, st4)
            if shellOut == True:
                shellPrint(st1)
                shellPrint(st2)
                shellPrint(st3)
                shellPrint(st4)
            if MQTTout:
                client.publish("gwh/tempBoard3", "{0:0.1f}".format(temperature3))
                client.publish("gwh/pressure3", "{0:0.2f}".format(pressure3))
                client.publish("gwh/pressureNN3", "{0:0.2f}".format(pressure_nn3))
                client.publish("gwh/altitude3", "{0:0.2f}".format(altitude3))
        except Exception as exc:
            exceptionPrint(exc, "BMP3xx Exception")

    if useLightSensor == True:
        try:
            lux = lightSensor.lux
            st1 = "Lux: {0:0.2f} lx".format(lux)
            if winOut == True:
                winPrint(tW, st1)
            if shellOut == True:
                shellPrint(st1)
            if MQTTout:
                client.publish("gwh/lux", "{0:0.2f}".format(lux))
        except Exception as exc:
            exceptionPrint(exc, "LightSensor Exception")
   
    if useHumiditySensor == True:
        try:
            tempHum = humiditySensor.temperature
            relHum = humiditySensor.relative_humidity
            st1 = "Temperatur: {0:0.1f} °C".format(tempHum)
            st2 = "Feuchte: {0:0.1f} %".format(relHum)
            if winOut == True:
                winPrint(tW, st1)
                winPrint(tW, st2)
            if shellOut == True:
                shellPrint(st1)
                shellPrint(st2)
            if MQTTout:
                client.publish("gwh/temp", "{0:0.1f}".format(tempHum))
                client.publish("gwh/humidity", "{0:0.1f}".format(relHum))
        except Exception as exc:
            exceptionPrint(exc, "Humidity Exception")
    
    if useSoilHumiditySensor == True:
        try:
            tempHum = soilHumiditySensor.temperature
            relHum = soilHumiditySensor.relative_humidity
            st1 = "Bodentemperatur: {0:0.1f} °C".format(tempHum)
            st2 = "Bodenfeuchte: {0:0.1f} %".format(relHum)
            if winOut == True:
                winPrint(tW, st1)
                winPrint(tW, st2)
            if shellOut == True:
                shellPrint(st1)
                shellPrint(st2)
            if MQTTout:
                client.publish("gwh/soiltemp", "{0:0.1f}".format(tempHum))
                client.publish("gwh/soilhumidity", "{0:0.1f}".format(relHum))
        except Exception as exc:
            exceptionPrint(exc, "Soil Humidity Exception")
    
    # T6703 CO2 Daten
    if useCO2Sensor == True:
        try:
            CO2value = CO2sensor.gasPPM()
            if CO2value > 0:
                if CO2value < CO2min:
                    CO2min = CO2value
                if CO2value > CO2max:
                    CO2max = CO2value
                st1 = "CO2 ppm: {0:5d}  min: {1:5d}  max: {2:5d}".format(CO2value, CO2min, CO2max)
                if winOut == True:
                    winPrint(tW, st1)
                if shellOut == True:
                    shellPrint(st1)
                if MQTTout:
                    client.publish("gwh/CO2", "{0:6d}".format(CO2value))
        except Exception as exc:
            exceptionPrint(exc, "CO2Sensor Exception")
    
    if useCCS811Sensor == True:
        try:
            eco2 = ccs811.eco2
            tvoc = ccs811.tvoc
            tempCCS = ccs811.temperature
            st1 = "eCO2: {0:5d} ppm".format(eco2)
            st2 = "TVOC: {0:5d} ppm".format(tvoc)
            st3 = "tempCCS: {0:0.1f} °C".format(tempCCS)
            if winOut == True:
                winPrint(tW, st1)
                winPrint(tW, st2)
                winPrint(tW, st3)
            if shellOut == True:
                shellPrint(st1)
                shellPrint(st2)
                shellPrint(st3)
            if MQTTout:
                client.publish("gwh/eCO2", "{0:5d}".format(eco2))
                client.publish("gwh/TVOC", "{0:5d}".format(tvoc))
                client.publish("gwh/tempCCS", "{0:0.1f} °C".format(tempCCS))
        except Exception as exc:
            exceptionPrint(exc, "CCS811 Exception")
 
    try:
    # O2 sensor
        vReference = 5.0  #3.3
        #value = O2sensor.value
        value = chanO2.value
        vOut = chanO2.voltage
        vOut = value * (vReference / (256.0*180))
        O2concentration = vOut * 0.21 / 2.05    # was 2.0
        if O2concentration < O2min:
            O2min = O2concentration
        if O2concentration > O2max:
            O2max = O2concentration
        st1 = "O2 value: {0:.4f}".format(vOut)
        st2 = "O2 %: {0:.3f}   min %: {1:.3f}  max %: {2:.3f}".format(O2concentration*100, O2min*100, O2max*100)
        if winOut == True:
            winPrint(tW, st1)
            winPrint(tW, st2)
        if shellOut == True:
            shellPrint(st1)
            shellPrint(st2)
        if MQTTout:
            client.publish("gwh/O2", "{0:0.3f}".format(O2concentration*100))
    except Exception as exc:
        exceptionPrint(exc, "AD O2 Exception")

    try:
        # Bodenfeuchte per Grove hat
        #value = SoilHumiditySensor.value        
        #print("Bodenfeuchte: {}".format(value))
        #value = SoilHumiditySensor2.value        
        #print("Bodenfeuchte2: {}".format(value))
        
        # soil humidity
        voltGas = chanGas.voltage
        valueGas = chanGas.value
        voltSoilHum = chanSoilHum.voltage
        valueSoilHum = chanSoilHum.value
        st1 = "Gas MQ-2: {:>5}\t{:>5.3f}".format(valueGas, voltGas)
            #print("soil2:    ", "{:>5}\t{:>5.3f}".format(chan2.value, chan2.voltage))
        st3 = "soil3:    {:>5}\t{:>5.3f}".format(valueSoilHum, voltSoilHum)
        if winOut == True:
            winPrint(tW, st1)
            winPrint(tW, st3)
        if shellOut == True:
            shellPrint(st1)
            shellPrint(st3)
        if MQTTout:
            client.publish("gwh/gas", "{:>5.3f}".format(voltGas))
            client.publish("gwh/soilHum", "{:>5.3f}".format(voltSoilHum))
    except Exception as exc:
        exceptionPrint(exc, "AD soil humidity Exception")

# 1-Wire Sensoren
    try:
        device_folders = glob.glob(base_dir + '28*')
        for device in device_folders:
            temp_s = "85.0"
            device_name = device[len(base_dir):]
            device_file = device + '/w1_slave'
            temp_c = read_temp(device_file)
            temp_s = "{0:3.3f}".format(temp_c)

            st1 = device_name + " {0} °C".format(temp_s)
            if winOut == True:
                winPrint(tW, st1)
            if shellOut == True:
                shellPrint(st1)
            if MQTTout:
                client.publish("gwh/{0}".format(device_name), temp_s)
    except Exception as exc:
        exceptionPrint(exc, "1-Wire Exception")
        
    #Trigger beachten - sollte low sein
        
    if ventFreshAir == True:
        if O2concentration > O2minforFreshAir + O2hysteresis:
            # Beende Frischluftzufuhr
            relais_1_status = 1
            ventFreshAir = False
        else:
            relais_1_status = 0
            ventFreshAir = True
    else:   #ventFreshAir == False
        if O2concentration > O2minforFreshAir:
            relais_1_status = 1
            ventFreshAir = False
        else:
            # Starte Frischluftzufuhr
            relais_1_status = 0
            ventFreshAir = True
                
    GPIO.output(relais_1_GPIO, relais_1_status)

    st1 = "ventilateFreshAir: {0:1d}".format(ventFreshAir)    
    if winOut == True:
        winPrint(tW, st1)
    if shellOut == True:
        print(st1)
    if MQTTout:
        if ventFreshAir:
            client.publish("gwh/ventFreshAir", "1")
        else:
            client.publish("gwh/ventFreshAir", "0")
 
    if airTemp > airTempCooling:
        relais_2_status = 1
        isCooling = False
    else:
        relais_2_status = 0
        isCooling = True
        
    GPIO.output(relais_2_GPIO, relais_2_status)
    st1 = "isCooling: {0:1d}".format(isCooling)  
    if winOut == True:
        winPrint(tW, st1)
    if shellOut == True:
        shellPrint(st1)
    if MQTTout:
        if isCooling:
            client.publish("gwh/isCooling", "1")
        else:
            client.publish("gwh/isCooling", "0")
    
    dtNow = datetime.datetime.now()
    tDelta = dtNow - dt

    secondsDiff = waitSeconds - tDelta.total_seconds()
    if secondsDiff > waitSeconds:
        secondsDiff = waitSeconds
    if secondsDiff < 0:
        secondsDiff = 0

    tDelta = dtNow - startTime
    tDeltaToReboot = datetime.timedelta(days=1)# 5; assert days > 0 to avoid endless reboot
    # print("tDelta ", tDelta)
    # print("tDeltaToReboot ", tDeltaToReboot)
    # print("dtNow.hour ", dtNow.hour)
    
    if tDelta > tDeltaToReboot:
        if dtNow.hour > 18:
            debugPrint("multisensor shutdown pending....")
            #os.system('sudo reboot -h now') #at evening

    st1 = "seconds2wait: {0:1f}".format(secondsDiff)
    if winOut == True:
        winPrint(tW, st1)
    if shellOut == True:
        shellPrint(st1)
        shellPrint('---')
    if winOut == True:
        root.update()
        #nada   #mainloop()
        
    time.sleep(secondsDiff)  #was 20

root.quit()
debugPrint("multisensor unerwartetes Ende!!!")
client.loop_stop()
client.disconnect()

#--fin-- 