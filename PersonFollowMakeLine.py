# import the necessary packages
from imutils.video import VideoStream
from imutils.video import FPS
import numpy as np
import argparse
import imutils
import time
import cv2
import serial
import threading
import json
#PASTE - python PersonFollow.py -p MobileNetSSD_deploy.prototxt.txt -m MobileNetSSD_deploy.caffemodel
#in terminal to run
#lets see if i can make output bigger
counter = 0
GeoList = list()

def rescale_frame(frame, percent=75):
    width = int(frame.shape[1] * percent/ 100)
    height = int(frame.shape[0] * percent/ 100)
    dim = (width, height)
    return cv2.resize(frame, dim, interpolation =cv2.INTER_AREA)

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-p", "--prototxt", default ="/home/gilblankenship/Projects/PythonCode/env/main/driveto/follow a line/MobileNetSSD_deploy.prototxt.txt",
    help="path to Caffe 'deploy' prototxt file")
ap.add_argument("-m", "--model", default ="/home/gilblankenship/Projects/PythonCode/env/main/driveto/follow a line/MobileNetSSD_deploy.caffemodel",
    help="path to Caffe pre-trained model")
ap.add_argument("-c", "--confidence", type=float, default=0.2,
    help="minimum probability to filter weak detections")
args = vars(ap.parse_args())

b = 7
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


#-----------------tracking-----------------------------

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


x = 500
y = 500
radius = 500




# initialize the list of class labels MobileNet SSD was trained to
# detect, then generate a set of bounding box colors for each class
CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
    "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
    "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
    "sofa", "train", "tvmonitor"]
COLORS = np.random.uniform(0, 255, size=(len(CLASSES), 3))

# load our serialized model from disk
print("[INFO] loading model...")
net = cv2.dnn.readNetFromCaffe(args["prototxt"], args["model"])
# initialize the video stream, allow the cammera sensor to warmup,
# and initialize the FPS counter
print("[INFO] starting video stream...")
vs = VideoStream(src=0).start()
time.sleep(2.0)
fps = FPS().start()

start_time = time.time()
# loop over the frames from the video stream
t1.start()
while True:
    # grab the frame from the threaded video stream and resize it
    # to have a maximum width of 400 pixels
    frame = vs.read()
    frame = imutils.resize(frame, width=400)

    # grab the frame dimensions and convert it to a blob
    (h, w) = frame.shape[:2]
    blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)),
        0.007843, (300, 300), 127.5)

    # pass the blob through the network and obtain the detections and
    # predictions
    net.setInput(blob)
    detections = net.forward()
    

# loop over the detections
    for i in np.arange(0, detections.shape[2]):
        # extract the confidence (i.e., probability) associated with
        # the prediction
        confidence = detections[0, 0, i, 2]

        # filter out weak detections by ensuring the `confidence` is
        # greater than the minimum confidence
        if confidence > args["confidence"]:
            # extract the index of the class label from the
            # `detections`, then compute the (x, y)-coordinates of
            # the bounding box for the object

            idx = int(detections[0, 0, i, 1])
            

            box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
            (startX, startY, endX, endY) = box.astype("int")
            if idx == 15:
                radius = endX - startX
                
                x = startX + (endX-startX)*(1/2)
                y = startY +(endY - startY)*(1/2)
            # draw the prediction on the frame
            label = "{}: {:.2f}%".format(CLASSES[idx],
                confidence * 100)
            cv2.rectangle(frame, (startX, startY), (endX, endY),
                COLORS[idx], 2)
            y = startY - 15 if startY - 15 > 15 else startY + 15
            cv2.putText(frame, label, (startX, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[idx], 2)



    if True:
        print(radius)
        if radius <200:
            if x > 275:
                if b!= 2:
                    Right()
                    b = 2
                    #print(2)
                    #time.sleep(1.3)
            if x <125:
                if b != 1:
                    Left()
                    b = 1
                    #print(1)
                    #time.sleep(1.3)
            if x > 125 and x < 275:
                if b != 0: 
                    Forward()
                    b = 0
                    #print(0)
                    #time.sleep(1.3)
                
        if radius >= 200:
            if b != 4:
                Stop()
                b = 4
                #print(4)
   









# # show the output frame
    # frame = rescale_frame(frame, 200)
    # cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF
    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break
    if time.time()-start_time > 45:
        break

    # update the FPS counter
    fps.update()
    
#     # stop the timer and display FPS information
stop_threads = True
with open('PersonLine.json', 'w') as f:
    json.dump(GeoList,f)
    f.close()

fps.stop()
print("[INFO] elapsed time: {:.2f}".format(fps.elapsed()))
print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()