
import time
#import smbus

from adafruit_extended_bus import ExtendedI2C as I2C
import adafruit_bno055

import multiprocessing as mp
from multiprocessing import Value
import serial
import geopy.distance

import json
from geographiclib.geodesic import Geodesic
counter = 0

with open('Line.json') as f:
   GeoList = [tuple(x) for x in json.load(f)]
locationlat, locationlon  = GeoList[counter]

arduino = serial.Serial(
    port = '/dev/ttyACM0',
    baudrate = 2000000, #perhaps make this lower need to do research
    bytesize = serial.EIGHTBITS,
    parity = serial.PARITY_NONE,
    stopbits = serial.STOPBITS_ONE,
    timeout = 5,
    xonxoff = False,
    rtscts = False,
    dsrdtr = False,
    writeTimeout = 2
    )

   




def Left():
    arduino.write("1".encode()) 
    print("left")
    global b
    b = 1
def Forward():
    arduino.write("0".encode())
    print("forward")
    global b
    b = 0
def Right():
    arduino.write("2".encode())
    print("right")
    global b
    b = 2
def Stop():
    arduino.write("4".encode())
    global b
    b = 4
def Back():
    arduino.write("3".encode())
    global b
    b = 3
def Search():
    arduino.write("5".encode())
    global b
    b = 5

b = 3

final = (locationlat, locationlon)


#input("press any key and enter to continue")
xx = Value('d',0.0)
yy = Value('d',0.0)
def begintrack():
    
    #global Geofencecoordinates
    # global xx    
    # global yy
    # global distance
    while True:
        try:
            with open('/home/gilblankenship/Projects/PythonCode/env/main/driveto/location.json') as json_file:
                locationdict = json.load(json_file)
            
            x = locationdict['latitude']
            xx.value = float(x)
            #print(x)
            y = locationdict['longetude']
            yy.value = float(y)
           
            #print(y)
            
            # current = (xx, yy)
            # distance = geopy.distance.distance(current,final).m
            time.sleep(.5)
        except json.decoder.JSONDecodeError as err:
            print(err)
            time.sleep(1)

         
            

track = mp.Process(target = begintrack)  

i2c = I2C(8)
sensor = adafruit_bno055.BNO055_I2C(i2c)
sensor.mode = adafruit_bno055.NDOF_MODE

last_val = 0xFFFF



yaw = Value('d',0.0)
currTime = time.time()
print_count = 0
def direction():
    global yangle
    global zangle
    global yaw
    while True:
        yaw.value, yangle, zangle = sensor.euler
        
        yaw.value = yaw.value - 10 #this accounts for magnetic vs true north  
        if yaw.value < 0:
            yaw.value = yaw.value + 360
        #print("yaw =",yaw.value)
        time.sleep(0.1)

dir = mp.Process(target = direction)

distance = Value('d',0.0)
def howfar():
    while True:
        global distance
        current = (xx.value, yy.value)
        distance.value = geopy.distance.distance(current,final).m
        #print("distance=",distance.value)

far = mp.Process(target = howfar)
track.start()
print("tracking")
time.sleep(5)
dir.start()
input("please move robot around in all directions, press enter when done")
#time.sleep(5)
print("finding direction")
h = sensor.calibration_status
print(h)
far.start()
time.sleep(5)
print("measuring distance")


while distance.value > 1:
    X = xx.value
    Y = yy.value
    bearing = Geodesic.WGS84.Inverse(X, Y, locationlat, locationlon)['azi1']
    #print(bearing)
    if bearing <0:
        bearing = bearing +360
    #print("bearing=",bearing)
    #print("yaw=",yaw.value)
    bearinglow = bearing - 13
    if bearinglow < 0:
        bearinglow = bearinglow + 360
    bearinghigh = bearing + 13
    if bearinghigh > 360:
        bearinghigh = bearinghigh - 360
    if yaw.value > bearinglow and yaw.value < bearinghigh and b != 0:
        Forward()
    if yaw.value < bearinglow  and b != 2:
        Right()
    if yaw.value > bearinghigh and b !=1:
        Left()
    # time.sleep(1)
    



if distance.value < 1:
    try:
        counter = counter +1
        locationlat, locationlon  = GeoList[counter]
    except IndexError:

        Stop()
        time.sleep(1)
        far.terminate()
        dir.terminate()
        track.terminate()
