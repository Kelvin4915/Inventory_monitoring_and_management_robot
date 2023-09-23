import cv2
import numpy as np
import RPi.GPIO as GPIO
import time
import cv2.aruco as aruco
import serial
# initialize the Pi Camera
cap = cv2.VideoCapture(0)
dir_check_pin = 23
dir_left_pin = 24
dir_right_pin = 25
aruco_check_pin = 6
aruco_done_pin = 5
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

# initialize the detector parameters
aruco_params = aruco.DetectorParameters_create()

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(dir_check_pin,GPIO.IN)
GPIO.setup(aruco_check_pin, GPIO.IN)
GPIO.setup(aruco_done_pin, GPIO.OUT)
GPIO.setup(aruco_done_pin, GPIO.LOW)
GPIO.setup(dir_left_pin,GPIO.OUT)
GPIO.setup(dir_right_pin,GPIO.OUT)
GPIO.output(dir_left_pin, GPIO.LOW)
GPIO.output(dir_right_pin, GPIO.LOW)
GPIO.output(aruco_done_pin, GPIO.LOW)

final_even =0
final_odd = 0

final = np.array([])
final2 = np.array([])

current_aruco_state = 0
already_in = 0

SERIAL_PORT = '/dev/serial0'
SERIAL_BAUDRATE = 9600
ser = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE)
ser.close()
ser.open()
while True:
    # capture frame-by-frame
    ret, frame = cap.read()
    dir_check = GPIO.input(dir_check_pin)
    aruco_check = GPIO.input(aruco_check_pin)
    last_aruco_state = current_aruco_state
    current_aruco_state = aruco_check

    #print(dir_check)
    #dir_check  = 1
    if(dir_check == 1):

        # convert to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # define range of blue color in HSV
        lower_blue = np.array([90, 50, 50])
        upper_blue = np.array([130, 255, 255])

        # threshold the HSV image to get only blue colors
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # apply morphological operations to reduce noise
        kernel = np.ones((5,5),np.uint8)
        mask = cv2.erode(mask,kernel,iterations = 1)
        mask = cv2.dilate(mask,kernel,iterations = 1)

        # find contours in the thresholded image
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # loop over the contours
        for cnt in contours:
            # approximate the contour as a polygon
            approx = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)
            # if the polygon has three sides
            if len(approx) == 3:
                # calculate the centroid of the triangle
                M = cv2.moments(approx)
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                # if the centroid is to the left of the frame, it's a left triangle
                x, y, w, h = cv2.boundingRect(approx)
                if cx < x + (w / 2):
                    cv2.drawContours(frame, [approx], 0, (0, 255, 0), 2)
                    cv2.putText(frame, 'LEFT TRIANGLE', (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
                    GPIO.output(dir_left_pin, GPIO.HIGH)
                    cv2.imshow('frame', frame)

                    time.sleep(2)
                    GPIO.output(dir_left_pin, GPIO.LOW)
                # if the centroid is to the right of the frame, it's a right triangle
                else:
                    cv2.drawContours(frame, [approx], 0, (0, 0, 255), 2)
                    cv2.putText(frame, 'RIGHT TRIANGLE', (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
                    GPIO.output(dir_right_pin, GPIO.HIGH)
                    cv2.imshow('frame', frame)
                    time.sleep(2)
                    GPIO.output(dir_right_pin, GPIO.LOW)                 
    
    elif (aruco_check == 1):
        
        # convert the image to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # detect the markers
        corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
        if (ids is not None):
            
            ids2 = ids.flatten()
            final = np.append(final,ids2)
            final2 = np.unique(final)
            final2int = final2.astype(int)
            final_even = np.sum(final2int%2 ==0)
            final_odd = np.sum(final2int%2 == 1)
#             print("Non-Defective Pieces : ",final_even)
#             print("Defective Pieces : ", final_odd)
            GPIO.output(aruco_done_pin, GPIO.HIGH)
            already_in = 1
            time.sleep(2)
#             GPIO.output(aruco_done_pin, GPIO.LOW)
            GPIO.output(aruco_done_pin, GPIO.LOW)


        
        # draw the detected markers on the frame
        frame_markers = aruco.drawDetectedMarkers(frame, corners, ids)
        
        # display the resulting frame
        cv2.imshow('frame', frame_markers)
    
            
    elif ((aruco_check == 0) and (already_in ==1)):
        print("Non-Defective Pieces : ",final_even)
        print("Defective Pieces : ", final_odd)
        print("total pieces detected:", len(final))
        already_in = 0
        string_1 = "Non Def :  "
        string_2 = " ::: Def : "
        ser.write(chr(12).encode())  # Command to clear the LCD display
        ser.write(chr(17).encode())
        ser.write(string_1.encode() + str(final_even).encode() + string_2.encode() + str(final_odd).encode())
        time.sleep(2)
        ser.write(chr(12).encode())
        ser.write(chr(18).encode())
        
        

        
            
            
        
        
        # wait for a key press and check if the 'q' key was pressed
    #if cv2.waitKey(1) & 0xFF == ord('q'):
    #    break

    
    
    # display the resulting frame
    
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# release the Pi Camera and close all windows
cap.release()
cv2.destroyAllWindows()