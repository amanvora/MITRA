import pyaudio
import cv2
import sys
import numpy as np
import math
import time
import pyttsx
import serial
import unicodedata
import speech_recognition as sr

engine = pyttsx.init()
engine.setProperty('rate', 120)
ser = serial.Serial(port = "COM12", baudrate=9600)
ser.close()
ser.open()
voices = engine.getProperty('voices')
sys.argv = ['','E:/VIII Sem/FYP/codes/Face Auth/face.jpg','E:/VIII Sem/FYP/codes/Face Auth/haarcascade_frontalface_alt.xml']
imagePath = sys.argv[1]
cascPath = sys.argv[2]

def speech():
    r = sr.Recognizer(language = "en-IN")
    with sr.Microphone() as source:
        while True:
            for voice in voices:
                engine.setProperty('voice', voice.id)
                print("Medicine?")
                engine.say("What's the name of the medicine?")
            engine.runAndWait()
            audio = r.record(source,duration=3)  # listen for the first phrase and extract it into audio data
            try:
                title = r.recognize(audio)
                title = unicodedata.normalize('NFKD', title).encode('ascii','ignore')
                print(title)    # recognize speech using Google Speech Recognition
                ser.write(title)
                ser.write('0')
                break;
            except LookupError:
                for voice in voices:
                    engine.setProperty('voice', voice.id)
                    engine.say("Could not understand audio. Please repeat")
                engine.runAndWait()
        while True:
            for voice in voices:
                engine.setProperty('voice', voice.id)
                print("Ward No.?")
                engine.say("What's the destination?")
            engine.runAndWait()
            audio = r.record(source,duration=3)  # listen for the first phrase and extract it into audio data
            try:
                title = r.recognize(audio)
                title = unicodedata.normalize('NFKD', title).encode('ascii','ignore')
                print(title)    # recognize speech using Google Speech Recognition
                if (title == '51'):
                    title = '0'
                if (title == '52'):
                    title = '1'
                if (title == '53'):
                    title = '2'                    
                ser.write(title)
                break;
            except LookupError:
                for voice in voices:
                    engine.setProperty('voice', voice.id)
                    engine.say("Could not understand audio. Please repeat")
                engine.runAndWait()                
                
cap = cv2.VideoCapture(0)
faceCascade = cv2.CascadeClassifier(cascPath)
fr_flag = 0
while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    camera = frame
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.equalizeHist(gray)    
    # Detect faces in the image
    faces = faceCascade.detectMultiScale(
    gray,
    scaleFactor = 1.3,
    minNeighbors = 5,
    minSize =(30, 30),
    flags = cv2.cv.CV_HAAR_SCALE_IMAGE
    )
    hmax = 0
    wmax = 0    
    # Draw a rectangle around the faces
    if (len(faces) >= 1):
        fr_flag = 1
        for (x, y, w, h) in faces:
            if h>hmax:
                hmax = h
                wmax = w
                store_x = x
                store_y = y
        x = store_x
        y = store_y
        h = hmax
        w = wmax   
        frame = gray[y:y+h , x:x+w]
        hs,ws = gray.shape
        if (hs!=0 and ws!=0):
            #cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            frame = cv2.resize(frame,(400,400))
            camera = camera[y:y+h,x:x+h]
            camera = cv2.resize(camera,(400,400))
            cv2.imshow("Faces found", camera)
            #cv2.imwrite("F"+str(1)+".jpg", frame)
            #cv2.imwrite("face.jpg",frame)
            imf = np.float32(frame)/255.0    # float conversion/scale
            dst = cv2.dct(imf)               # the dct
            img_dct = dst[0:100,0:100]
            img_gal = cv2.imread("E:/VIII Sem/FYP/codes/Face Auth/face.jpg")
            img_gal = cv2.cvtColor(img_gal, cv2.COLOR_BGR2GRAY)
            img_gal = cv2.resize(img_gal,(400,400))
            imf = np.float32(img_gal)/255.0    # float conversion/scale
            dst = cv2.dct(imf)               # the dct
            img_gal_dct = dst[0:100,0:100]
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            dist = (img_gal_dct - img_dct)**2
            sum1 = 0
            for j in range(0,100):
                for k in range(0,100):
                    sum1 += (dist[j,k])
            dist = math.sqrt(sum1)
            print dist
            time.sleep(2)
            if dist<75:
                cap.release()
                cv2.destroyAllWindows()
                print "Authorized person."
                for voice in voices:
                    engine.setProperty('voice', voice.id)
                    engine.say("Authorized person identified.")
                engine.runAndWait()                
                speech()
                break
            else:
                cv2.destroyAllWindows()
                print "Unauthorized person."
                for voice in voices:
                    engine.setProperty('voice', voice.id)
                    engine.say("Unauthorized person.")
                engine.runAndWait()                                
                break
