#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import PointCloud
import ros_numpy as np
import matplotlib.pyplot as plt
import levenberg_marquardt as LM

# 전역변수 선언
zero_setting_flag = 1           # 4~5번 뒤에 offset을 작동시키기 위한 flag변수 선언
zero_Val = [    [0, 0, 0],
                [0, 0, 0],
                [0, 0, 0],
                [0, 0, 0],
                [0, 0, 0],
                [0, 0, 0],
                [0, 0, 0],
                [0, 0, 0],
                [0, 0, 0] ]     # offset을 위해 배경 노이즈값을 저장할 리스트 변수 선언



# 함수 선언

# 본격적인 node함수
def callback1x(data): 
    global zero_Val
    global zero_setting_flag

    for i in range(9):  # 9번 반복하면서 i번째 센서 값 출력
        print(round((data.points[i].x)-zero_Val[i][0], 4), round((data.points[i].y)-zero_Val[i][1], 4), round((data.points[i].z)-zero_Val[i][2], 4))
    print('-----------------------------------------')
    print('\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n')

    zero_setting_flag *= 2          # offset flag 변수에 x2 누산

    if (zero_setting_flag > 16.0):  # flag변수가 16보다 커지면 offset 함수 호출
        zero_setting(data)


# offset을 위해 배경 노이즈 값을 리스트에 저장하는 함수
def zero_setting(data):
    global zero_Val
    global zero_setting_flag

    for i in range(9):               # 9번 반복문
        zero_Val[i][0] = data.points[i].x
        zero_Val[i][1] = data.points[i].y
        zero_Val[i][2] = data.points[i].z   # i번째 센서에서 측정하는 배경 노이즈 값을 저장
    zero_setting_flag = 0                   # 이후 x2 누산되어 아무때나 함수가 호출되지 않도록 0으로 초기화



# 메인 함수
def main():      

    rospy.init_node('algorithm_pkg_node', anonymous=True)
    rospy.Subscriber('local_sys', PointCloud, callback1x)

    rospy.spin()    # node 무한 반복



if __name__ == '__main__':
    main()                  # main문 무한 호출
    
from sensor_msgs.msg import PointCloud