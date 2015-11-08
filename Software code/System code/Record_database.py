import cv2
import sys
import numpy as np
import time
sys.argv = ['','E:/VIII Sem/FYP/codes/Face Auth/face.jpg','E:/VIII Sem/FYP/codes/Face Auth/haarcascade_frontalface_alt.xml']
imagePath = sys.argv[1]
cascPath = sys.argv[2]

cap = cv2.VideoCapture(0)
faceCascade = cv2.CascadeClassifier(cascPath)
i=0
while(True):
    i += 1
    # Capture frame-by-frame
    ret, frame = cap.read()
    camera = frame
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.equalizeHist(gray)
    # Detect faces in the image
    faces = faceCascade.detectMultiScale(
    gray,
    scaleFactor=1.3,
    minNeighbors=5,
    minSize=(30, 30),
    flags = cv2.cv.CV_HAAR_SCALE_IMAGE
    )
    # Draw a rectangle around the faces
    if (len(faces) == 1):
        for (x, y, w, h) in faces:
            frame = gray[y:y+h , x:x+w]
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.imshow("Camera",camera)
            hs,ws = frame.shape
            if (hs!=0 and ws!=0):
                frame = cv2.resize(frame,(400,400)) 
                #cv2.imwrite("F"+str(1)+".jpg", frame)
                camera = camera[y:y+h,x:x+h]
                camera = cv2.resize(camera,(400,400))                 
                cv2.imwrite("face.jpg",camera)
                cv2.imshow("Faces found", camera)
                imf = np.float32(frame)/255.0    # float conversion/scale
                dst = cv2.dct(imf)               # the dct
                img_dct = np.uint8(dst)*255.0    # convert back          
                img_dct = img_dct[0:200,0:200]
                cv2.imshow("Cropped DCT of face", img_dct)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
