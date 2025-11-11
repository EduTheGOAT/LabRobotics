# LabRobotics
Repo to host all code and examples for robotics labs


#Control proporcional
 import WebGUI

import HAL

import Frequency

import cv2


KP = 0.0095

V_MAX = 10.0

V_MIN = 4.0

IMG_CENTER = 320


i = 0

while True:

img = HAL.getImage()

hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

red_mask = cv2.inRange(hsv,

(0, 125, 125),

(30,255,255))

contours, hierarchy = cv2.findContours(red_mask,

cv2.RETR_TREE,

cv2.CHAIN_APPROX_SIMPLE)

if len(contours) > 0:

c = max(contours, key=cv2.contourArea)

M = cv2.moments(c)


if M["m00"] != 0:

cX = M["m10"] / M["m00"]

err = IMG_CENTER - cX

v_dyn = V_MAX - (abs(err) / IMG_CENTER) * (V_MAX - V_MIN)

w_dyn = KP * err


HAL.setV(v_dyn)

HAL.setW(w_dyn)

else:

HAL.setV(0)

HAL.setW(0)

else:

HAL.setV(0)

HAL.setW(0)

WebGUI.showImage(img)

print(i)

i+=1 


#Control derivativo

import WebGUI
import HAL
import Frequency
import cv2


KP = 0.009
KD = 0.0085

V_MAX = 10.0
V_MIN = 5.0

IMG_CENTER = 320

last_err = 0

while True:
    img = HAL.getImage()
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    red_mask = cv2.inRange(hsv, (0, 125, 125), (30,255,255))
    contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        M = cv2.moments(c)

        if M["m00"] != 0:
            cX = M["m10"] / M["m00"]
            err = IMG_CENTER - cX

            derivative = err - last_err

            v_dyn = V_MAX - ((abs(err) / IMG_CENTER) * (V_MAX - V_MIN))
            w_dyn = KP * err + KD * derivative

            HAL.setV(v_dyn)
            HAL.setW(w_dyn)

            last_err = err  
        else:
            HAL.setV(0)
            HAL.setW(0)
    else:
        HAL.setV(0)
        HAL.setW(0)
        
    WebGUI.showImage(img)



  #Control integrativo

  import WebGUI
import HAL
import Frequency
import cv2


KP = 0.0075
KI = 0.0000002
KD = 0.0085

V_MAX = 8.0
V_MIN = 4.0

IMG_CENTER = 320

integral = 0.0
last_err = 0.0

while True:
    img = HAL.getImage()
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    red_mask = cv2.inRange(hsv, (0, 125, 125), (30,255,255))
    contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        M = cv2.moments(c)

        if M["m00"] != 0:
            cX = M["m10"] / M["m00"]
            err = IMG_CENTER - cX

            integral = integral + err
            derivative = err - last_err

            v_dyn = V_MAX - ((abs(err) / IMG_CENTER) * (V_MAX - V_MIN))
            w_dyn = (KP * err) + (KI * integral) + (KD * derivative)

            HAL.setV(v_dyn)
            HAL.setW(w_dyn)

            last_err = err  
        else:
            HAL.setV(0)
            HAL.setW(0)
            integral = 0.0
            last_err = 0.0
    else:
        HAL.setV(0)
        HAL.setW(0)
        integral = 0.0
        last_err = 0.0
        
    WebGUI.showImage(img)
