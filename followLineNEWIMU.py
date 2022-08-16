
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
m = 1 #get set to zero to kill whole program
b = 3 #counter 
counter = 0
locationlat = Value('d',0.0)
locationlon = Value('d',0.0)
with open('Line.json') as f:
   GeoList = [tuple(x) for x in json.load(f)]
locationlat.value, locationlon.value  = GeoList[counter]
print(GeoList)
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
            json_file.close()
            x = locationdict['latitude']
            xx.value = float(x)
            #print(x)
            y = locationdict['longetude']
            yy.value = float(y)
           
            #print(y)
            
            # current = (xx, yy)
            # distance = geopy.distance.distance(current,final).m
            
        except json.decoder.JSONDecodeError as err:
            #print(err)
            time.sleep(.01)

         
            

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
        #time.sleep(0.1)

dir = mp.Process(target = direction)

distance = Value('d',0.0)
def howfar():
    while True:
        global distance
        final = (locationlat.value, locationlon.value)
        current = (xx.value, yy.value)
        distance.value = geopy.distance.distance(current,final).m
        #print(distance.value)
        #print("distance=",distance.value)

far = mp.Process(target = howfar)
track.start()
print("tracking")
time.sleep(3)
dir.start()
input("please move robot around in all directions, press enter when done")
#time.sleep(5)
print("finding direction")
h = sensor.calibration_status
print(h)
far.start()
time.sleep(5)
print("measuring distance")

while True:
    while distance.value > 1:
        #print(distance.value)
        X = xx.value
        Y = yy.value
        bearing = Geodesic.WGS84.Inverse(X, Y, locationlat.value, locationlon.value)['azi1']
        #print(bearing)
        if bearing <0:
            bearing = bearing + 360
        if bearing >360:
            bearing = bearing - 360
        #print("bearing=",bearing)
        #print("yaw=",yaw.value)
        bearinglow = bearing - 20
        if bearinglow < 0:
            bearinglow = bearinglow + 360
        bearinghigh = bearing + 20
        if bearinghigh > 360:
            bearinghigh = bearinghigh - 360
        
        if bearinglow < bearinghigh:
            if yaw.value >= bearinglow and yaw.value <= bearinghigh and b != 0:
                Forward()
            elif yaw.value < bearinglow  and b != 2:
                Right()
            elif yaw.value > bearinghigh and b !=1:
                
                Left()
                # time.sleep(1)
            else:
                pass
        elif bearinglow > bearinghigh:
            if yaw.value >= bearinghigh and yaw.value <= bearinglow and b != 0:
                Forward()
            elif yaw.value < bearinghigh  and b != 2:
                Right()
            elif yaw.value > bearinglow and b !=1:
                Left()
            else:
                pass
        time.sleep(.1)
        

        



    while distance.value <= 1:
        try:
            Stop()
            print(counter, "point reached")
            counter = counter +1
            locationlat.value, locationlon.value  = GeoList[counter]
            
            time.sleep(2)
            break #may be unnecessary
            
        except IndexError:
            m = 0
            print('Final point reached')

            Stop()
            time.sleep(1)
            far.terminate()
            dir.terminate()
            track.terminate()
            distance.value = 0
    if m == 0:
        break
        
