# -*- coding: utf-8 -*-

import platform
import numpy as np
import argparse
import cv2
import serial
import time
import sys
from threading import Thread
import csv

import math


X_255_point = 0
Y_255_point = 0
X_Size = 0
Y_Size = 0
Area = 0
Angle = 0
#-----------------------------------------------
Top_name = 'mini CTS5 setting'
hsv_Lower = 0
hsv_Upper = 0

hsv_Lower0 = 0
hsv_Upper0 = 0

hsv_Lower1 = 0
hsv_Upper1 = 0

rect_point = 0

'''
ag1 = 30    # 로보베이직 측에서 보낸 값에 따라 각도를 판단할 경우
ag2 = 45    # In this code: 직접 각도 값을 시리얼 통신으로 제공한다고 상정
ag3 = 60
'''

readRX = 0 # 머리 각도 값_from robobasic

#----------- 
color_num = [   0,  1,  2,  3,  4]
    
h_max =     [ 255, 243,196,111,110]
h_min =     [  55,  0,158, 59, 74]
    
s_max =     [ 162,110,223,110,255]
s_min =     [ 114, 0,150, 51,133]
    
v_max =     [  77,246,239,156,255]
v_min =     [   0,110,104, 61,104]
    
min_area =  [  50, 50, 50, 10, 10]

certain_area_1 = [1000, 2000, 2500, 1500, 1500] # 로봇이 보행을 멈출 시기를 판단하기 위한 공의 넓이 최소버전
certain_area_2 = [1000, 2000, 2500, 1500, 1500] # 로봇이 보행을 멈출 시기를 판단하기 위한 공의 넓이 최대버전

now_color = 0
serial_use = 1

serial_port =  None
Temp_count = 0
Read_RX =  0

mx,my = 0,0

required_area = [250, 400] #화면 상에 공이 있어야 하는 위치. x, y

threading_Time = 5/1000.

Config_File_Name ='Cts5_v1.dat'

ball_detection_area = 0
location_ball_coordi = -1

for_cnt = 0 # count of adjusting flag pos and ball pos
#-----------------------------------------------

def nothing(x):
    pass

#-----------------------------------------------
def create_blank(width, height, rgb_color=(0, 0, 0)):

    image = np.zeros((height, width, 3), np.uint8)
    color = tuple(reversed(rgb_color))
    image[:] = color

    return image
#-----------------------------------------------
def draw_str2(dst, target, s):
    x, y = target
    cv2.putText(dst, s, (x+1, y+1), cv2.FONT_HERSHEY_PLAIN, 0.8, (0, 0, 0), thickness = 2, lineType=cv2.LINE_AA)
    cv2.putText(dst, s, (x, y), cv2.FONT_HERSHEY_PLAIN, 0.8, (255, 255, 255), lineType=cv2.LINE_AA)
#-----------------------------------------------
def draw_str3(dst, target, s):
    x, y = target
    cv2.putText(dst, s, (x+1, y+1), cv2.FONT_HERSHEY_PLAIN, 1.5, (0, 0, 0), thickness = 2, lineType=cv2.LINE_AA)
    cv2.putText(dst, s, (x, y), cv2.FONT_HERSHEY_PLAIN, 1.5, (255, 255, 255), lineType=cv2.LINE_AA)
#-----------------------------------------------
def draw_str_height(dst, target, s, height):
    x, y = target
    cv2.putText(dst, s, (x+1, y+1), cv2.FONT_HERSHEY_PLAIN, height, (0, 0, 0), thickness = 2, lineType=cv2.LINE_AA)
    cv2.putText(dst, s, (x, y), cv2.FONT_HERSHEY_PLAIN, height, (255, 255, 255), lineType=cv2.LINE_AA)
#-----------------------------------------------
def clock():
    return cv2.getTickCount() / cv2.getTickFrequency()
#-----------------------------------------------

def Trackbar_change(now_color):
    global  hsv_Lower,  hsv_Upper
    hsv_Lower = (h_min[now_color], s_min[now_color], v_min[now_color])
    hsv_Upper = (h_max[now_color], s_max[now_color], v_max[now_color])

#-----------------------------------------------
def Hmax_change(a):
    
    h_max[now_color] = cv2.getTrackbarPos('Hmax', Top_name)
    Trackbar_change(now_color)
#-----------------------------------------------
def Hmin_change(a):
    
    h_min[now_color] = cv2.getTrackbarPos('Hmin', Top_name)
    Trackbar_change(now_color)
#-----------------------------------------------
def Smax_change(a):
    
    s_max[now_color] = cv2.getTrackbarPos('Smax', Top_name)
    Trackbar_change(now_color)
#-----------------------------------------------
def Smin_change(a):
    
    s_min[now_color] = cv2.getTrackbarPos('Smin', Top_name)
    Trackbar_change(now_color)
#-----------------------------------------------
def Vmax_change(a):
    
    v_max[now_color] = cv2.getTrackbarPos('Vmax', Top_name)
    Trackbar_change(now_color)
#-----------------------------------------------
def Vmin_change(a):
    
    v_min[now_color] = cv2.getTrackbarPos('Vmin', Top_name)
    Trackbar_change(now_color)
#-----------------------------------------------
def min_area_change(a):
   
    min_area[now_color] = cv2.getTrackbarPos('Min_Area', Top_name)
    if min_area[now_color] == 0:
        min_area[now_color] = 1
        cv2.setTrackbarPos('Min_Area', Top_name, min_area[now_color])
    Trackbar_change(now_color)
#-----------------------------------------------
def Color_num_change(a):
    global now_color, hsv_Lower,  hsv_Upper
    now_color = cv2.getTrackbarPos('Color_num', Top_name)
    cv2.setTrackbarPos('Hmax', Top_name, h_max[now_color])
    cv2.setTrackbarPos('Hmin', Top_name, h_min[now_color])
    cv2.setTrackbarPos('Smax', Top_name, s_max[now_color])
    cv2.setTrackbarPos('Smin', Top_name, s_min[now_color])
    cv2.setTrackbarPos('Vmax', Top_name, v_max[now_color])
    cv2.setTrackbarPos('Vmin', Top_name, v_min[now_color])
    cv2.setTrackbarPos('Min_Area', Top_name, min_area[now_color])

    hsv_Lower = (h_min[now_color], s_min[now_color], v_min[now_color])
    hsv_Upper = (h_max[now_color], s_max[now_color], v_max[now_color])
#----------------------------------------------- 
def TX_data(ser, one_byte):  # one_byte= 0~255
    # ser.write(chr(int(one_byte)))          #python2.7
    ser.write(serial.to_bytes([one_byte]))  #python3
#-----------------------------------------------
def RX_data(serial):
    global Temp_count
    try:
        if serial.inWaiting() > 0:
            result = serial.read(1)
            RX = ord(result)
            return RX
        else:
            return 0
    except:
        Temp_count = Temp_count  + 1
        print("Serial Not Open " + str(Temp_count))
        return 0
        pass
#-----------------------------------------------

#*************************
# mouse callback function
def mouse_move(event,x,y,flags,param):
    global mx, my

    if event == cv2.EVENT_MOUSEMOVE:
        mx, my = x, y


# # *************************
def RX_Receiving(ser):
    global receiving_exit,threading_Time

    global X_255_point
    global Y_255_point
    global X_Size
    global Y_Size
    global Area, Angle


    receiving_exit = 1
    while True:
        if receiving_exit == 0:
            break
        time.sleep(threading_Time)
        
        while ser.inWaiting() > 0:
            result = ser.read(1)
            RX = ord(result)
            print ("RX=" + str(RX))
            

def GetLengthTwoPoints(XY_Point1, XY_Point2):
    return math.sqrt( (XY_Point2[0] - XY_Point1[0])**2 + (XY_Point2[1] - XY_Point1[1])**2 )
# *************************
def FYtand(dec_val_v ,dec_val_h):
    return ( math.atan2(dec_val_v, dec_val_h) * (180.0 / math.pi))
# *************************
#degree 값을 라디안 값으로 변환하는 함수
def FYrtd(rad_val ):
    return  (rad_val * (180.0 / math.pi))

# *************************
# 라디안값을 degree 값으로 변환하는 함수
def FYdtr(dec_val):
    return  (dec_val / 180.0 * math.pi)

# *************************
def GetAngleTwoPoints(XY_Point1, XY_Point2):
    xDiff = XY_Point2[0] - XY_Point1[0]
    yDiff = XY_Point2[1] - XY_Point1[1]
    cal = math.degrees(math.atan2(yDiff, xDiff)) + 90
    if cal > 90:
        cal =  cal - 180
    return  cal
# *************************
def TurnHN(a):       # Turn_Humanoid. 화면을 4사분면으로 나누고 공의 위치가 어느 사분면에 있는지 파악
    return 0        
    

    

#************************
import math

def cal_distance(a, b):
    # a = 로봇의 높이, b = 로봇 머리 각도 (도 단위)
    distance = a * math.tan(b)
    return distance
#distance = cal_distance(a, b) #이 구문으로 구현
#************************
def hsv_setting_save():

    global Config_File_Name, color_num
    global color_num, h_max, h_min 
    global s_max, s_min, v_max, v_min, min_area
    
    try:
    #if 1:
        saveFile = open(Config_File_Name, 'w')
        i = 0
        color_cnt = len(color_num)
        while i < color_cnt:
            text = str(color_num[i]) + ","
            text = text + str(h_max[i]) + "," + str(h_min[i]) + ","
            text = text + str(s_max[i]) + "," + str(s_min[i]) + ","
            text = text + str(v_max[i]) + "," + str(v_min[i]) + ","
            text = text + str(min_area[i])  + "\n"
            saveFile.writelines(text)
            i = i + 1
        saveFile.close()
        print("hsv_setting_save OK")
        return 1
    except:
        print("hsv_setting_save Error~")
        return 0
    

    
#************************
def hsv_setting_read():
    global Config_File_Name
    global color_num, h_max, h_min 
    global s_max, s_min, v_max, v_min, min_area

    try:
        if 1:
            with open(Config_File_Name) as csvfile:
                readCSV = csv.reader(csvfile, delimiter=',')
                i = 0
                
                for row in readCSV:
                    color_num[i] = int(row[0])
                    h_max[i] = int(row[1])
                    h_min[i] = int(row[2])
                    s_max[i] = int(row[3])
                    s_min[i] = int(row[4])
                    v_max[i] = int(row[5])
                    v_min[i] = int(row[6])
                    min_area[i] = int(row[7])
                    
                    i = i + 1
                
            csvfile.close()
            print("hsv_setting_read OK")
        return 1
    except:
       print("hsv_setting_read Error~")
       return 0

### 시리얼 신호 5프레임당 한번 송출

def _serial(serial_port, num, str=" "):
    global serial_count

    while(1):
        serial_count+=1
        # print("serial_count : ", serial_count)
        if 50 < serial_count & serial_count < 56:
            if serial_count%5 == 0:
                print(str, "TX_DATA", num)
                print("send tx data")
                TX_data(serial_port, num)
                serial_count = 0

        else:
            break
         
def ball_detect_area(frame, cnts, now_color, ball_detection_area):
    global ball_count
    now_color = 0
    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        ((X, Y), radius) = cv2.minEnclosingCircle(c)
        ball_count += 1
        x4, y4, w4, h4 = cv2.boundingRect(c)

        cv2.rectangle(frame, (x4, y4), (x4 + w4, y4 + h4), (0, 255, 0), 2)
        Area = cv2.contourArea(c) / min_area[now_color]
        if Area > 255:
            Area = 255

        if Area > min_area[now_color]:
            x4, y4, w4, h4 = cv2.boundingRect(c)
            ## 사각형 그리기
            ## 중점 위치로 머리의 위, 아래 명령 판단
            cv2.rectangle(frame, (x4, y4), (x4 + w4, y4 + h4), (0, 255, 0), 2)
            rect_point_x = x4 + w4 / 2 
            rect_point_y = y4 + h4 / 2
            rect_point = (rect_point_x, rect_point_y)  # 사각형 중앙 좌표
            
             # if ball_count > 20:
            print("ball_detection", ball_detection, now_color)
            if now_color == 0 & ball_detection == 0:
                if Area < 50 :
                    print("send serial")
                    _serial(serial_port, 12, str="area up")
                    serial_count = 0
                    #TX_data(serial_port, 11)  # Go. # 전송값: 임의. 수정 필요
                    # time.sleep(0.01) 
                    print("area up")
                elif Area > 150:
                    print("send serial")
                    #TX_data(serial_port, 12)  #뒤로 Go
                    # time.sleep(0.01)
                    _serial(serial_port, 11, str="area down")
                    serial_count = 0
                # 50 < Area < 150
                else:
                    print("done")
                    ball_detection_area = 1
                    ball_count = 0
    else:
        x = 0
        y = 0
        X_255_point = 0
        Y_255_point = 0
        X_Size = 0
        Y_Size = 0
        Area = 0
        Angle = 0
        ball_detection_area = 0

        _serial(serial_port, 7, str="cannot find ball => left turn") ## 왼쪽 턴
        #_serial(serial_port, 9, str="right turn") ## 오른쪽 턴
        serial_count = 0
    return ball_detection_area

def ball_detect_loc(frame, cnts, now_color, ball_detection_area, ball_count, golf_flag):
    golf_flag = 0
    ball_count += 1
                    
    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        ((X, Y), radius) = cv2.minEnclosingCircle(c)
        x4, y4, w4, h4 = cv2.boundingRect(c)
        ## 사각형 그리기
        ## 중점 위치로 머리의 위, 아래 명령 판단
        cv2.rectangle(frame, (x4, y4), (x4 + w4, y4 + h4), (0, 255, 0), 2)
        rect_point_x = x4 + w4 / 2
        rect_point_y = y4 + h4 / 2
        rect_point = (rect_point_x, rect_point_y)  # 사각형 중앙 좌표
        if ball_detection_area == 1:
            if now_color == 0:
            ## 공이 위치해야할 범위 800 500
                cv2.rectangle(frame, (450, 300), (350, 200), (255, 255, 0), 2)
                print("cccccc")
            if (350 > rect_point_x):
                print("=================move left 14 gogo=================")
                _serial(serial_port, 15, str="move left") ## 왼쪽으로 이동
                
                serial_count = 0
                golf_flag = 0


            elif (450 < rect_point_x):
                _serial(serial_port, 20, str="move right") ## 오른쪽으로 이동
                serial_count = 0
                golf_flag = 0
                            
            else:
                if (250 > rect_point_y):
                    _serial(serial_port, 10, str="go forward")
                     ## 앞으로 이동
                    serial_count = 0
                    golf_flag = 0
                                
                elif (300 < rect_point_y):
                    _serial(serial_port, 32, str="go back")
                     ## 뒤로 이동
                    serial_count = 0
                    golf_flag = 0

                                
                else:
                    print("golf shot", golf_flag)
                    golf_flag = 1
                    ball_count += 1
                    now_color = 1
                    serial_count = 0
                    
    
    return golf_flag
#**************************************************
    
def flag_detect(frame, cnts, now_color):
    global flag_time_count
    flag_detected = 0
    print("aaaa")

    # left_flag_exist = 0
    # right_flag_exist = 0
    # front_flag_exist = 0

    if len(cnts) > 0:
        print("bbbbbb")
        c = max(cnts, key=cv2.contourArea)
        ((X, Y), radius) = cv2.minEnclosingCircle(c)

        Area = cv2.contourArea(c) / min_area[now_color]
        Area = min(Area, 255)

        if Area > min_area[now_color]:
            print("cccccccccccccccccccccccccc")
            x4, y4, w4, h4 = cv2.boundingRect(c)
            cv2.rectangle(frame, (x4, y4), (x4 + w4, y4 + h4), (0, 255, 0), 2)
            # if flag_time_count < 50:
            #     left_flag_exist = 1
            #     print("left flag")
            # elif 50 < flag_time_count < 100:
            #     front_flag_exist = 1
            #     print("front flag")
            # elif 100 < flag_time_count < 150:
            #     right_flag_exist = 1
            #     print("right flag")
            flag_detected = 1
            _serial(serial_port, 8, str="golf shot pos") ## 왼쪽샷위치_골프 2
            serial_count = 0
    else:
        x = 0
        y = 0
        X_255_point = 0
        Y_255_point = 0
        X_Size = 0
        Y_Size = 0
        Area = 0
        Angle = 0
        flag_detected = 0
        _serial(serial_port, 7, str="left turn") ## 왼쪽 턴
        #_serial(serial_port, 9, str="right turn") ## 오른쪽 턴
        
        TX_data (serial_port, 7) # 시계방향으로 돌기
        serial_count = 0
    return flag_detected

#**************************************************
### 깃발을 카메라 영상의 특정 위치로 정렬
def flag_adjust(frame, cnts, now_color, flag_adjusting):
    global flag_count
    flag_adjusting = 0

    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        ((X, Y), radius) = cv2.minEnclosingCircle(c)
        x4, y4, w4, h4 = cv2.boundingRect(c)
        ## 사각형 그리기
        ## 중점 위치로 머리의 위, 아래 명령 판단
        cv2.rectangle(frame, (x4, y4), (x4 + w4, y4 + h4), (0, 255, 0), 2)
        rect_point_x = x4 + w4 / 2
        rect_point_y = y4 + h4 / 2
        rect_point = (rect_point_x, rect_point_y)  # 사각형 중앙 좌표
        if now_color == 1:
            ## 깃발이 위치해야할 범위 800 500
            cv2.rectangle(frame, (450, 300), (350, 200), (255, 255, 0), 2)
        ## 깃발과 로봇 위치 조정
        if (350 > rect_point_x):
            _serial(serial_port, 15, str="move left") ## 왼쪽 이동
            serial_count = 0

        elif (450 < rect_point_x):
            _serial(serial_port, 20, str="move right") ## 오른쪽 이동
            serial_count = 0                        
        else:
            if (230 > rect_point_y):
                _serial(serial_port, 10, str="go forward") ## 전진
                serial_count = 0                           
            elif (300 < rect_point_y):
                _serial(serial_port, 32, str="go back") ## 후진
                serial_count = 0                            
            else:
                print("golf flag detected", golf_flag)
                _serial(serial_port, 8, str="golf shot pos") ## 왼쪽샷위치_골프 2

                flag_adjusting = 1
                flag_count += 1
                
    return flag_adjusting

# **************************************************
if __name__ == '__main__':
    #-------------------------------------
    print ("-------------------------------------")
    print ("(2020-1-20) mini CTS5 Program.  MINIROBOT Corp.")
    print ("-------------------------------------")
    print ("")
    os_version = platform.platform()
    print (" ---> OS " + os_version)
    python_version = ".".join(map(str, sys.version_info[:3]))
    print (" ---> Python " + python_version)
    opencv_version = cv2.__version__
    print (" ---> OpenCV  " + opencv_version)
    
   
   
    #-------------------------------------
    #---- user Setting -------------------
    #-------------------------------------
    W_View_size =  800  #320  #640
    #H_View_size = int(W_View_size / 1.777)
    # H_View_size = int(W_View_size / 1.333)
    H_View_size = 500

    BPS =  4800  # 4800,9600,14400, 19200,28800, 57600, 115200
    serial_use = 1
    now_color = 0
    View_select = 0
    #-------------------------------------
    print(" ---> Camera View: " + str(W_View_size) + " x " + str(H_View_size) )
    print ("")
    print ("-------------------------------------")
    
    #-------------------------------------
    try:
        hsv_setting_read()
    except:
        hsv_setting_save()
        
        
    #-------------------------------------
    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--video",
                    help="path to the (optional) video file")
    ap.add_argument("-b", "--buffer", type=int, default=64,
                    help="max buffer size")
    args = vars(ap.parse_args())

    img = create_blank(320, 100, rgb_color=(0, 0, 255))
    
    cv2.namedWindow(Top_name)
    cv2.moveWindow(Top_name,0,0)
    
    cv2.createTrackbar('Hmax', Top_name, h_max[now_color], 255, Hmax_change)
    cv2.createTrackbar('Hmin', Top_name, h_min[now_color], 255, Hmin_change)
    cv2.createTrackbar('Smax', Top_name, s_max[now_color], 255, Smax_change)
    cv2.createTrackbar('Smin', Top_name, s_min[now_color], 255, Smin_change)
    cv2.createTrackbar('Vmax', Top_name, v_max[now_color], 255, Vmax_change)
    cv2.createTrackbar('Vmin', Top_name, v_min[now_color], 255, Vmin_change)
    cv2.createTrackbar('Min_Area', Top_name, min_area[now_color], 255, min_area_change)
    cv2.createTrackbar('Color_num', Top_name,color_num[now_color], 4, Color_num_change)

    Trackbar_change(now_color)

    draw_str3(img, (15, 25), 'MINIROBOT Corp.')
    draw_str2(img, (15, 45), 'space: Fast <=> Video and Mask.')
    draw_str2(img, (15, 65), 's, S: Setting File Save')
    draw_str2(img, (15, 85), 'Esc: Program Exit')
    
    
    cv2.imshow(Top_name, img)
    #---------------------------
    if not args.get("video", False):
        camera = cv2.VideoCapture(0)
    else:
        camera = cv2.VideoCapture(args["video"])
    #---------------------------
    camera.set(3, W_View_size)
    camera.set(4, H_View_size)
    camera.set(5, 40)
    time.sleep(0.5)
    #---------------------------
    
   
    
    #---------------------------
    (grabbed, frame) = camera.read()
    draw_str2(frame, (5, 15), 'X_Center x Y_Center =  Area' )
    draw_str2(frame, (5, H_View_size - 5), 'View: %.1d x %.1d.  Space: Fast <=> Video and Mask.'
                      % (W_View_size, H_View_size))
    draw_str_height(frame, (5, int(H_View_size/2)), 'Fast operation...', 3.0 )
    mask = frame.copy()
    cv2.imshow('mini CTS5 - Video', frame )
    cv2.imshow('mini CTS5 - Mask', mask)
    cv2.moveWindow('mini CTS5 - Mask',322 + W_View_size,36)
    cv2.moveWindow('mini CTS5 - Video',322,36)
    cv2.setMouseCallback('mini CTS5 - Video', mouse_move)

    #---------------------------
    if serial_use != 0:  # python3
    #if serial_use <> 0:  # python2.7
        BPS =  4800  # 4800,9600,14400, 19200,28800, 57600, 115200
        #---------local Serial Port : ttyS0 --------
        #---------USB Serial Port : ttyAMA0 --------
        serial_port = serial.Serial('/dev/ttyS0', BPS, timeout=0.01)
        serial_port.flush() # serial cls
        time.sleep(0.2)
    
        serial_t = Thread(target=RX_Receiving, args=(serial_port,))
        serial_t.daemon = True
        serial_t.start()
        
    #First -> Start Code Send 
    TX_data(serial_port, 250)
    TX_data(serial_port, 250)
    TX_data(serial_port, 250)
    
    old_time = clock()

    View_select = 0
    msg_one_view = 0
    ball_detection = 0
    ball_count = 0
    golf = 0
    flag_adjusting = 0
    flag_time_count = 0
    flag_count = 0
    serial_count = 0
    golf_flag = 0
    now_color = 0
    
    flag_detected = 0
    final = 0
    final_ball_detect = 0
    final_flag_adjust = 0
    final_ball_flag_num = 0
    
    # -------- Main Loop Start --------
    while True:

        # grab the current frame
        (grabbed, frame) = camera.read()

        if args.get("video") and not grabbed:
            break

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2YUV)  # HSV => YUV
        mask = cv2.inRange(hsv, hsv_Lower, hsv_Upper)
        
        hsv_Lower = (h_min[now_color], s_min[now_color], v_min[now_color])
        hsv_Upper = (h_max[now_color], s_max[now_color], v_max[now_color])
        
        '''
        mask0 = cv2.inRange(hsv, (h_min[0], s_min[0], v_min[0]), (h_max[0], s_max[0], v_max[0]))
        mask1 = cv2.inRange(hsv, (h_min[1], s_min[1], v_min[1]), (h_max[1], s_max[1], v_max[1]))
        mask2 = cv2.inRange(hsv, (h_min[2], s_min[2], v_min[2]), (h_max[2], s_max[2], v_max[2]))
        mask3 = cv2.inRange(hsv, (h_min[3], s_min[3], v_min[3]), (h_max[3], s_max[3], v_max[3]))
        mask4 = cv2.inRange(hsv, (h_min[4], s_min[4], v_min[4]), (h_max[4], s_max[4], v_max[4]))
        '''
        
        
        #mask = cv2.erode(mask, None, iterations=1)
        #mask = cv2.dilate(mask, None, iterations=1)
        #mask = cv2.GaussianBlur(mask, (3, 3), 2)  # softly

        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        
        '''
        cnts0 = cv2.findContours(mask0.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        cnts1 = cv2.findContours(mask1.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        cnts2 = cv2.findContours(mask2.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        cnts3 = cv2.findContours(mask3.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        cnts4 = cv2.findContours(mask4.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        '''
        # left_flag_exist = 1
        # right_flag_exist = 0
        # front_flag_exist = 0
        #flag_adjusting = 0

        center = None

        # i = 0
        # now_color = 0
        # ball_detection_area = 0
        # golf_flag = 0
        #### 골프공의 거리 맞추기
#=====================================================================#
######################## 깃발 테스트 코드
        # now_color = 1
        # flag_detect(frame, cnts, now_color)
        # flag_time_count+=1
#=====================================================================#

        #print("now_color", now_color)
        
        
        if (ball_detection_area == 0):
            ball_detection_area = ball_detect_area(frame, cnts, now_color, ball_detection_area)
            golf_flag = 0


            #print("ball_detection_area : ", ball_detection_area)
        #### 로봇과 골프공의 거리 맞추기, golf_flag == 0이면 ball pos
        elif (ball_detection_area == 1 and golf_flag == 0):
            now_color = 0
            golf_flag = ball_detect_loc(frame, cnts, now_color, ball_detection_area, ball_count, golf_flag)
            
            print("ball adjusting")
            print("=== glof_flag : ", golf_flag, " ===")
            
        #### 깃발 위치 별로 골프 자세 수행 후 샷
        elif (ball_detection_area == 1 and golf_flag == 1):
            now_color = 1
            flag_adjusting = flag_detect(frame, cnts, now_color)
            print("finding flag", flag_adjusting)
            if flag_adjusting  == 1:
                TX_data(serial_port, 8)
                print("flag adjusted", flag_adjusting)
            #if (flag_detected == 1):
            #    flag_time_count+=1
            # if (left_flag_exist == 1 and right_flag_exist ==0 and front_flag_exist==0):
            #     ball_count += 1
            #     for for_count in range(0, 10):
            #         if (golf==0 and flag_adjusting==0):
                        
            #             #왼쪽 보는 키로 #TX_data 전송 및 고개 들기

            #             ball_count, flag_adjusting= flag_adjust(frame, cnts, now_color, ball_count)
            #             # 정면 보는 키로 #TX_data 전송 및 고개 내리기
            #             golf = ball_detect_loc(frame, cnts, now_color, ball_detection_area, ball_count, golf_flag)
            #             for_count += 1
            #         elif (golf==0 and flag_adjusting==1):
            #             # 정면 보는 키로 #TX_data 전송 및 고개 내리기
            #             golf = ball_detect_loc(frame, cnts, now_color, ball_detection_area, ball_count, golf_flag)
            #             for_count += 1
            #         else:
            #             for_count += 1
            #             ball_count += 1
            #         if ball_count > 300:
            #         # 정면 보는 키로 #TX_data 전송 및 고개 내리기
            #         ##TX_data(serial_port, 2)
            #             golf=0
            #             flag_adjusting=0
            #             for_count=0 
            #             ball_detection=0 
            #             golf_flag=0 
            #             ball_count=0 
            #             ball_detection_area = 0
            #     # Stop
                        
                    
            # elif (left_flag_exist == 0 and right_flag_exist == 1 and front_flag_exist==0):
                
            #     ball_count += 1
            #     for for_count in range(0, 10):
            #         if (golf==0 and flag_adjusting==0):
            #             #왼쪽 보는 키로 #TX_data 전송 및 고개 들기
                        
            #             ball_count, flag_adjusting= flag_adjust(frame, cnts, now_color, ball_count)
            #             # 정면 보는 키로 #TX_data 전송 및 고개 내리기
                        
            #             golf = ball_detect_loc(frame, cnts, now_color, ball_detection_area, ball_count, golf_flag)
            #             for_count += 1
            #         elif (golf==0 and flag_adjusting==1):
            #             print("in elif", flag_adjusting, ball_count)
            #             # 정면 보는 키로 #TX_data 전송 및 고개 내리기
            #             golf = ball_detect_loc(frame, cnts, now_color, ball_detection_area, ball_count, golf_flag)
            #             for_count += 1
            #         else:
            #             for_count += 1
            #             ball_count += 1
            #         if ball_count > 300:
            #         # 정면 보는 키로 #TX_data 전송 및 고개 내리기
            #         ##TX_data(serial_port, 2)
            #             golf=0
            #             flag_adjusting=0
            #             for_count=0 
            #             ball_detection=0 
            #             golf_flag=0 
            #             ball_count=0 
            #             ball_detection_area = 0
                        
                    
            #elif (left_flag_exist == 0 and right_flag_exist == 0 and front_flag_exist==1):
            if (flag_adjusting == 1):
                flag_time_count+=1
                ball_count_now = 0
                print("========== start ball & flag & robot all final adjust==========")
                final = 1
                print("final : ", final)
                #################################
        if final == 1:
            print("final : ", final)
            now_color = 0
            final_ball_detect = ball_detect_loc(frame, cnts, now_color, ball_detection_area, ball_count, golf_flag)
            ### after golf position, ball detection ok
            if (final_ball_flag_num<10):
                if final_ball_detect == 1:
                    _serial(serial_port, 17, str="head left")
                    now_color = 1
                    final_flag_adjust = flag_adjust(frame, cnts, now_color, ball_count)
                    if final_flag_adjust == 1:
                        _serial(serial_port, 34, str="head position to ball")
                        final_ball_detect = 0
                        final_flag_adjust = 0
                        final_ball_flag_num+=1
            else:
                _serial(serial_port, 2, str="=== final golf ===")
                print("~~~~~ all cleared ~~~~~")
                ball_detection = 0
                ball_count = 0
                golf = 0
                flag_adjusting = 0
                flag_time_count = 0
                flag_count = 0
                serial_count = 0
                golf_flag = 0
                now_color = 0
    
                flag_detected = 0
                final = 0
                final_ball_detect = 0
                final_flag_adjust = 0
                final_ball_flag_num = 0
                        
                
                        #왼쪽 보는 키로 #TX_data 전송 및 고개 들기
                        #TX_data(serial_port, 34)
                        #TX_data(serial_port, 17)
                        #_serial(serial_port, 34)
                        #serial_count = 0
            '''
                    _serial(serial_port, str=17)
                    serial_count = 0
                        flag_adjusting= flag_adjust(frame, cnts, now_color, ball_count)
                        # 정면 보는 키로 #TX_data 전송 및 고개 내리기
                        TX_data(serial_port, 29)
                        #_serial(serial_port, 29)
                        serial_count = 0
                
                        golf = ball_detect_loc(frame, cnts, now_color, ball_detection_area, ball_count, golf_flag)
                        print("ball & flag adjusting")
                    elif (golf==0 and flag_adjusting==1):
                        # 정면 보는 키로 #TX_data 전송 및 고개 내리기
                        #TX_data(serial_port, 29)
                        _serial(serial_port, 29)
                        serial_count = 0
                        golf = ball_detect_loc(frame, cnts, now_color, ball_detection_area, ball_count, golf_flag)
            '''
                    #else:
                    #    print("~~~~~ all cleared ~~~~~")
                    # 정면 보는 키로 #TX_data 전송 및 고개 내리기
                        #TX_data(serial_port, 29)
                        #TX_data(serial_port, 2)
            '''
                        _serial(serial_port, 29)
                        serial_count = 0
                        _serial(serial_port, 2)
                        serial_count = 0
                        golf=0
                        #flag_adjusting=0
                        for_count=0 
                        ball_detection=0 
                        golf_flag=0 
                        ball_count=0 
                        ball_detection_area = 0
                        go_to_golf_flag = 0
                        now_color = 0
            '''
            
        
                
       
        Frame_time = (clock() - old_time) * 1000.
        old_time = clock()

           
        if View_select == 0: # Fast operation 
            print(" " + str(W_View_size) + " x " + str(H_View_size) + " =  %.1f ms" % (Frame_time ))
            #temp = Read_RX
            pass
            
        elif View_select == 1: # Debug
            
            if msg_one_view > 0:
                msg_one_view = msg_one_view + 1
                cv2.putText(frame, "SAVE!", (50, int(H_View_size / 2)),
                            cv2.FONT_HERSHEY_PLAIN, 5, (255, 255, 255), thickness=5)
                
                if msg_one_view > 10:
                    msg_one_view = 0                
                                
            draw_str2(frame, (3, 15), 'X: %.1d, Y: %.1d, Area: %.1d' % (X_255_point, Y_255_point, Area))
            draw_str2(frame, (3, H_View_size - 5), 'View: %.1d x %.1d Time: %.1f ms  Space: Fast <=> Video and Mask.'
                      % (W_View_size, H_View_size, Frame_time))
                      
            #------mouse pixel hsv -------------------------------
            mx2 = mx
            my2 = my
            if mx2 < W_View_size and my2 < H_View_size:
                pixel = hsv[my2, mx2]
                set_H = pixel[0]
                set_S = pixel[1]
                set_V = pixel[2]
                pixel2 = frame[my2, mx2]
                if my2 < (H_View_size / 2):
                    if mx2 < (W_View_size / 2):
                        x_p = -30
                    elif mx2 > (W_View_size / 2):
                        x_p = 60
                    else:
                        x_p = 30
                    draw_str2(frame, (mx2 - x_p, my2 + 15), '-HSV-')
                    draw_str2(frame, (mx2 - x_p, my2 + 30), '%.1d' % (pixel[0]))
                    draw_str2(frame, (mx2 - x_p, my2 + 45), '%.1d' % (pixel[1]))
                    draw_str2(frame, (mx2 - x_p, my2 + 60), '%.1d' % (pixel[2]))
                else:
                    if mx2 < (W_View_size / 2):
                        x_p = -30
                    elif mx2 > (W_View_size / 2):
                        x_p = 60
                    else:
                        x_p = 30
                    draw_str2(frame, (mx2 - x_p, my2 - 60), '-HSV-')
                    draw_str2(frame, (mx2 - x_p, my2 - 45), '%.1d' % (pixel[0]))
                    draw_str2(frame, (mx2 - x_p, my2 - 30), '%.1d' % (pixel[1]))
                    draw_str2(frame, (mx2 - x_p, my2 - 15), '%.1d' % (pixel[2]))
            #----------------------------------------------
            
            cv2.imshow('mini CTS5 - Video', frame )
            cv2.imshow('mini CTS5 - Mask', mask)

        key = 0xFF & cv2.waitKey(1)
        
        if key == 27:  # ESC  Key
            break
        elif key == ord(' '):  # spacebar Key
            if View_select == 0:
                View_select = 1
            else:
                View_select = 0
        elif key == ord('s') or key == ord('S'):  # s or S Key:  Setting valus Save
            hsv_setting_save()
            msg_one_view = 1

    # cleanup the camera and close any open windows
    receiving_exit = 0
    time.sleep(0.5)
    
    camera.release()
    cv2.destroyAllWindows() 
