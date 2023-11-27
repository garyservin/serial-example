#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import PointCloud
import numpy as np
import matplotlib.pyplot as plt
import levenberg_marquardt as LM

# 전역변수 선언
zero_setting_flag = 0           # 4~5번 뒤에 offset을 작동시키기 위한 flag변수 선언
array_Val = np.array([  [0, 0, 0],
                        [0, 0, 0],
                        [0, 0, 0],
                        [0, 0, 0],
                        [0, 0, 0],
                        [0, 0, 0],
                        [0, 0, 0],
                        [0, 0, 0],
                        [0, 0, 0] ])     # sensor 값들의 numpy조작을 간편하게 하기 위해
                                         # 옮겨 저장할 배열 변수 선언
zero_Val = np.array([   [0, 0, 0],
                        [0, 0, 0],
                        [0, 0, 0],
                        [0, 0, 0],
                        [0, 0, 0],
                        [0, 0, 0],
                        [0, 0, 0],
                        [0, 0, 0],
                        [0, 0, 0] ])     # offset을 위해 배경 노이즈값을 저장할 배열 변수 선언



# 함수 선언

# 본격적인 node함수
def callback1x(data): 
    global array_Val
    global zero_Val
    global zero_setting_flag

    # numpy를 사용하기 위해 list 값을 array로 이동
    for i in range(9):
        array_Val[i][0] = data.points[i].x
        array_Val[i][1] = data.points[i].y
        array_Val[i][2] = data.points[i].z

    
    '''
    for i in range(9):  # 9번 반복하면서 i번째 센서 값 출력
        print(round((data.points[i].x), 4), 
              round((data.points[i].y), 4), 
              round((data.points[i].z), 4))
    '''
    
    print('-----------------------------------------')


    #zero_setting_flag *= 2          # offset flag 변수에 x2 누산

    if (zero_setting_flag == 0):  # flag변수가 16보다 커지면 offset 함수 호출
        zero_setting()

    offset_Setting()
    print('\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n')




# offset을 위해 배경 노이즈 값을 리스트에 저장하는 함수
def zero_setting():
    global array_Val
    global zero_Val
    global zero_setting_flag

    for i in range(9):               # 9번 반복문
        zero_Val[i][0] = array_Val[i][0]
        zero_Val[i][1] = array_Val[i][1]
        zero_Val[i][2] = array_Val[i][2]   # i번째 센서에서 측정하는 배경 노이즈 값을 저장

    zero_setting_flag = 1                   # 이후 x2 누산되어 아무때나 함수가 호출되지 않도록 0으로 초기화



# offset 계산하는 함수
def offset_Setting():
    global array_Val
    global zero_Val

    sensor_avg = array_Val.mean(axis=0)  # 각 축 성분들의 평균 값 계산
    '''
    # 계산한 평균 값을 각 센서 값에서부터 뺀 후에 출력
    for i in range(9):  # 9번 반복하면서 i번째 센서 값 출력
        print(round((array_Val[i][0])-sensor_avg[0], 4), 
              round((array_Val[i][1])-sensor_avg[1], 4), 
              round((array_Val[i][2])-sensor_avg[2], 4))
    '''
    for i in range(9):  # 9번 반복하면서 i번째 센서 값 출력
        print(round((array_Val[i][0])-zero_Val[i][0], 4), 
              round((array_Val[i][1])-zero_Val[i][1], 4), 
              round((array_Val[i][2])-zero_Val[i][2], 4))




# 메인 함수
def main():      

    rospy.init_node('algorithm_pkg_node', anonymous=True)
    rospy.Subscriber('local_sys', PointCloud, callback1x)

    rospy.spin()    # node 무한 반복



if __name__ == '__main__':
    main()                  # main문 무한 호출
    
from sensor_msgs.msg import PointCloud