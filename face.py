import numpy as np
import cv2
from time import *
from math import *

cascade_filename = 'haarcascade_frontalface_alt.xml'
cascade = cv2.CascadeClassifier(cascade_filename)
cam = cv2.VideoCapture(1)    

xc = 320
yc = 240
dx = 0
dy = 0

def robot_move(vec):
    dx = 0.1*(1 if vec[0]>0 else -1)
    dy = 0.1*(1 if vec[1]>0 else -1)
    return dx,dy


if cam.isOpened():                      
    while True:
        ret, img = cam.read()    
        start_x, start_y, width, height = 160, 0, 320, 480
        end_x, end_y = start_x + width, start_y + height
        cropped_image = img[start_y:end_y, start_x:end_x]

        if ret:
            # img = cv2.resize(img,dsize=None,fx=0.75,fy=0.75) 
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            results = cascade.detectMultiScale(gray,scaleFactor=1.1,minNeighbors=5,minSize=(30,30))
            if len(results)>0:
                x, y, w, h = results[0]
                xf = int(x+w/2)
                yf = int(y+h/2)
                
                vec = np.array([xf-xc, yf-yc])
                print(vec)
                dx, dy = robot_move(vec)
                print(dx,dy)
                if cv2.waitKey(1) == ord('s'):
                    cv2.imwrite('cheese.jpg', cropped_image)
                    print('taken')
                    break
                # if (np.linalg.norm(vec)<5):
                #     print("cheese")
                #     sleep(1)
                    

                #     cv2.imwrite('cheese.jpg', cropped_image)
                #     break 
                cv2.arrowedLine(img, (xc,yc),(xf,yf),(0,0,0), 2) 
            cv2.circle(img, (xc,yc), 5, (0,0,0), 2)
            cv2.line(img, (160, 0), (160, 480), (0,255,0))
            cv2.line(img, (480, 0), (480, 480), (0,255,0))
            cv2.imshow('camera', img)           
            if cv2.waitKey(1) == ord('q'):   
                break                   
        else:
            print('no frame')
            break
else:
    print("can't open camera.")
cam.release()                           # 자원 반납
cv2.destroyAllWindows()