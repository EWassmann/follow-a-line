# importing all of the needed packages
from shapely.geometry import Polygon, Point
import multiprocessing as mp
import serial
from ublox_gps import UbloxGps
import threading
import time
import multiprocessing as mp
from multiprocessing import Value
import json
#setting up the port and baudrate for the gps (serial communications), baudrate is what sparkfun reccomended on github

#initalizing the counter for the Geofence coordinats list as well as the Geofence coordinates list
counter = 0
GeoList = list()

#setting up the serial communications with the arduino
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
#these are all the functions that write to the arduino that tell it to move, the numbers are sent as a 
#string and the arduino will decode them on that side
#----------------Functions---------------------------
def Left():
    arduino.write("1".encode()) 
    global b
    b = 1
def Forward():
    arduino.write("0".encode())
    global b
    b = 0
def Right():
    arduino.write("2".encode())
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
#this is the inital thread it is used to store the gps coordinates of a track the robot takes, and assighn them to a list containing tuples that 
#shapely can use


def track(stop):
    print("Beginning to track")
    while True:
        try:
            global counter
            global GeoList
            
        
            with open('/home/gilblankenship/Projects/PythonCode/env/main/driveto/location.json') as json_file:
                locationdict = json.load(json_file)
            json_file.close()
            x = locationdict['latitude']
            #print(x)
            y = locationdict['longetude']

            GeoList.insert(counter, (x,y))
            
            counter = counter + 1
        except json.decoder.JSONDecodeError as err:
            print(err)
            print("in json")
            #this breaks out of the function
        time.sleep(3)
        
#starting the thread for creating the fence
stop_threads = False
t1 = threading.Thread(target = track, args = (lambda: stop_threads, ))
t1.start()

#this is the second thread it just keeps track of the robots positon so part two of the code can see if it is in or out of the fence

 
#loop for making the fence
while True:
    
        
    

    print("Use WASD to move, q to end program")


    directions = input()
    #stuff below is from keyboard controll it is looking for the inputs to run the movement functions from the top


    if directions == "w":
        Forward()
    if directions == "a":
        Left()
    if directions =="d":
        Right()
    if directions == "s":
        Stop()
    if directions == "ss":
        Back() 
   
    if directions== "q":
        
        stop_threads = True
        with open('Line.json', 'w') as f:
            json.dump(GeoList,f)
            f.close()
        break
        


#starting the second thread
