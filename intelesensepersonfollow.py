# import the necessary packages
# from imutils.video import VideoStream
# from imutils.video import FPS
import pyrealsense2 as rs
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

# def rescale_frame(frame, percent=75):
#     width = int(frame.shape[1] * percent/ 100)
#     height = int(frame.shape[0] * percent/ 100)
#     dim = (width, height)
#     return cv2.resize(frame, dim, interpolation =cv2.INTER_AREA)

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# construct the argument parse and parse the arguments (modified with defaults so the script does not need to be ran in command line)
ap = argparse.ArgumentParser()
ap.add_argument("-p", "--prototxt", default ="/home/gilblankenship/Projects/PythonCode/env/main/driveto/follow a line/MobileNetSSD_deploy.prototxt.txt",
    help="path to Caffe 'deploy' prototxt file")
ap.add_argument("-m", "--model", default ="/home/gilblankenship/Projects/PythonCode/env/main/driveto/follow a line/MobileNetSSD_deploy.caffemodel",
    help="path to Caffe pre-trained model")
ap.add_argument("-c", "--confidence", type=float, default=0.7,
    help="minimum probability to filter weak detections")
args = vars(ap.parse_args())

b = 7 #keeps track of last direction information given so that the arduino is not written more than one of the same instructions
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

#----------------Functions---------------
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
            xpos = locationdict['latitude']
            #print(x)
            ypos = locationdict['longetude']

            GeoList.insert(counter, (xpos,ypos))
            
            counter = counter + 1
        except json.decoder.JSONDecodeError as err:
            print(err)
            print("in json")
            #this breaks out of the function
        time.sleep(3)
        
#starting the thread for creating the fence
stop_threads = False
t1 = threading.Thread(target = track, args = (lambda: stop_threads, ))

#initalizing the center of the detection so no errors are thrown and it does not cause robot to turn one way or another
xx = 700
yy = 500





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
# vs = VideoStream(src=1).start()
# time.sleep(2.0)
# fps = FPS().start()
distave = 0
start_time = time.time()
start_time2 = time.time()
# loop over the frames from the video stream
t1.start()
pipeline.start(config)
while True:
    # grab the frame from the threaded video stream and resize it
    # to have a maximum width of 400 pixels
    frame = pipeline.wait_for_frames()
    depth_frame = frame.get_depth_frame()
    color_frame = frame.get_color_frame()
    if not depth_frame or not color_frame:
        continue

    # grab the frame dimensions and convert it to a blob
    h = color_frame.get_height()
    
    w = color_frame.get_width()
    
    color_image = np.asanyarray(color_frame.get_data())
    depth_image = np.asanyarray(depth_frame.get_data())
    blob = cv2.dnn.blobFromImage(cv2.resize(color_image, (300, 300)),
        0.007843, (300, 300), 127.5)
   
    

    # pass the blob through the network and obtain the detections and
    # predictions
    net.setInput(blob)
    detections = net.forward()
    olddist = distave

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
            
                xx = int(startX + (endX-startX)*(1/2))
                yy = int(startY +(endY - startY)*(1/2))
                disttot = 0
                for a in range (xx - 5, xx + 5):
                    for g in range (yy-5, yy+5):
                        disttot = disttot + depth_frame.get_distance(a,g)
                distave = disttot/100
            
                

                
            
            # draw the prediction on the frame
            label = "{}: {:.2f}%".format(CLASSES[idx],
                confidence * 100)
            cv2.rectangle(color_image, (startX, startY), (endX, endY),
                COLORS[idx], 2)
            y = startY - 15 if startY - 15 > 15 else startY + 15
            cv2.putText(color_image, label, (startX, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[idx], 2)
    #below, if it has been over 4 seconds since it noticed a person it will set distave to zero, if it has been less than four seconds it will
    #keep doing whatever it was doing before
    if distave != olddist:
        start_time2 = time.time()
    if distave == olddist:
        if time.time() - start_time2 > 4:
            distave = 0
            



    
        
    if distave >1.5:
        
        if xx > 420:
            if b!= 2:
                Right()
                b = 2
                #print(2)
                #time.sleep(1.3)
        if xx <200:
            if b != 1:
                Left()
                b = 1
                #print(1)
                #time.sleep(1.3)
        if xx > 200 and xx < 420:
            if b != 0: 
                Forward()
                b = 0
                #print(0)
                    
    elif distave < 1.5:
        if b != 4:
            Stop()
            b = 4
            #print(4)
            #print(distave)
    else:
        if b!=4:
            Stop()
            b = 4
            #print(4)
            #print(distave)










# # show the output frame
    #frame = rescale_frame(frame, 200)
    cv2.imshow("Frame", color_image)
    key = cv2.waitKey(1) & 0xFF
    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break
    # if time.time()-start_time > 45:
    #    Stop()
    #     break

    
    
#     # stop the timer and display FPS information
stop_threads = True
with open('PersonLine.json', 'w') as f:
    json.dump(GeoList,f)
    f.close()

#fps.stop()

# do a bit of cleanup
pipeline.stop()
cv2.destroyAllWindows()
