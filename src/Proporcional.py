import HAL
import Frequency
import cv2
import WebGUI

KP = 0.0095
V_MAX = 10.0
V_MIN = 4.0
IMG_CENTER = 320

while True:
    img = HAL.getImage()
    
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    red_mask = cv2.inRange(hsv,
                           (0, 125, 125),
                           (30, 255, 255))
    
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
