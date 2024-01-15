#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import PointCloud
from std_msgs.msg import String
from scipy.optimize import leastsq
import numpy as np
import pprint
import matplotlib.pyplot as plt
import levenberg_marquardt as LM

# 전역변수 선언
zero_setting_flag = 0           # 4~5번 뒤에 offset을 작동시키기 위한 flag변수 선언
#first_pos = np.array([0, -100, 40])
first_pos = np.array([118, 118, 65])
first_h = np.array([0.77, -0.06, 0.45])
full_packet = ""                # 패킷 값을 저장하는 리스트 변수
sensor_data = []                # 해체작업을 진행할 패킷 값을 저장하는 리스트 변수
packet_count = 0                       # 분할되어 들어오는 패킷을 총 10번만 받게 하기 위해 카운트를 세는 변수
is_collecting = False

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

P = np.array([                           # hall sensor의 각 위치좌표 값 초기화(좌표 단위는 mm)
                [-118,  118,   0],
                [   0,  118,   0],
                [ 118,  118,   0],
                [-118,    0,   0],
                [   0,    0,   0],      ## 5번 센서로, 중앙이 되는 센서다
                [ 118,    0,   0],
                [-118, -118,   0],
                [   0, -118,   0],
                [ 118, -118,   0] ])

# 상수 선언
MU0 = 4*(np.pi)*10**(-7)    # 진공투자율[mT/A]
MU = 1.056                  # 사용하는 자석의 투자율[mT/A]
MU_R = MU/MU0               # 사용하는 자석의 상대투자율[mT/A]
M0 = 1.012                  # 사용하는 자석의 등급에 따른 값[T]: 실험에서 사용하는 자석 등급은 N42
M_T = (np.pi)*(4.7625**2)*(12.7)*(1.012)    # 자석의 자화벡터 값 = pi*(반지름^2)*(높이)*(자석 등급)









# 함수 선언

# Serial_example_node.cpp를 통해 받은 패킷 값을 분해하는 함수
def seperating_Packet(data):
    global full_packet, packet_count, is_collecting, sensor_data

    # 'Z'로 시작하는 패킷을 감지하거나 이미 패킷 수집 중인 경우
    if data.data.startswith('Z') or is_collecting:
        full_packet += data.data
        packet_count += 1

        # 패킷 수집 시작
        if data.data.startswith('Z'):
            is_collecting = True

        # 10개의 패킷을 수집한 경우
        if packet_count == 10:
            #print(full_packet)
            print(parse_packet(full_packet))
            full_packet = ""        # 패킷 초기화
            packet_count = 0        # 분할 패킷 카운트 초기화
            is_collecting = False   # 분할 패킷 저장 flag 초기화
            
    


#패킷 분해 함수에서 부호를 결정해주는 함수 
def parse_value(value_str):
    sign = -1 if value_str[0] == '1' else 1
    return sign * int(value_str[1:])

# 2차원으로 저장되어 있는 리스트 변수 깔끔하게 출력해주는 함수
def pretty_print(data):
    for row in data:
        print(" ".join("{:>10.2f}".format(x) for x in row))

# 본격적인 센서 패킷 값 분해 코드
def parse_packet(packet):
    if not packet.startswith("ZZ"):
        return None

    raw_sum = 0  # 원본 센서 값들의 합
    sensors_data = []
    for char in range(ord('A'), ord('I') + 1):
        start_char = chr(char)
        end_char = chr(char + 32)
        start_idx = packet.find(start_char) + 1
        end_idx = packet.find(end_char)
        sensor_str = packet[start_idx:end_idx]

        sensor_values = [parse_value(value) for value in sensor_str.split(',')]
        raw_sum += sum(sensor_values)  # 가공 전 원본 데이터 합산
        sensor_values[0] *= -1
        sensor_values = [v / 100.0 for v in sensor_values]

        sensors_data.append(sensor_values)

    checksum_str = packet[packet.find('i') + 1:packet.find('Y')]
    checksum = parse_value(checksum_str)

    if raw_sum != checksum:
        return None

    pretty_print(sensors_data)
    #return sensors_data





# 단순히 offset 설정을 위한 flag 변수 값을 조정하는 함수
def callback_offset(data):
    global zero_setting_flag

    zero_setting_flag = 0


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
    
    #print('-----------------------------------------')


    if (zero_setting_flag == 0):  # flag변수가 0이면 offset 함수 호출
        zero_setting()

    offset_Setting()              # offset 설정하는 함수 호출




# offset을 위해 배경 노이즈 값을 리스트에 저장하는 함수
def zero_setting():
    global array_Val
    global zero_Val
    global zero_setting_flag

    for i in range(9):               # 9번 반복문
        zero_Val[i][0] = array_Val[i][0]
        zero_Val[i][1] = array_Val[i][1]
        zero_Val[i][2] = array_Val[i][2]   # i번째 센서에서 측정하는 배경 노이즈 값을 저장

    zero_setting_flag = 1                   # 이후 1로 설정하여 함수가 호출되지 않도록 초기화
                                            # 이렇게 안 하면 센서 측정 값이 계속 0으로만 출력됨



# offset 계산하는 함수
def offset_Setting():
    global array_Val
    global zero_Val
    global first_pos, first_h, differences

    h_val = np.array([0,0,0,0,0,0,0,0,0])

    sensor_avg = array_Val.mean(axis=0)  # 각 축 성분들의 평균 값 계산
    '''
    # 계산한 평균 값을 각 센서 값에서부터 뺀 후에 출력
    for i in range(9):  # 9번 반복하면서 i번째 센서 값 출력
        array_Val[i][0]-sensor_avg[0]
        array_Val[i][1]-sensor_avg[1]
        array_Val[i][2]-sensor_avg[2]
    '''
    
    for i in range(9):  # i번째 센서 배열에 배경 노이즈를 뺀 값을 저장 
        array_Val[i][0] -= zero_Val[i][0]/1000
        array_Val[i][1] -= zero_Val[i][1]/1000
        array_Val[i][2] -= zero_Val[i][2]/1000

    # 각 센서에서 측정한 자기장 값의 norm을 계산
    for i in range(9):
        h_val[i] = np.sqrt( (array_Val[i][0] ** 2) + 
                            (array_Val[i][1] ** 2) + 
                            (array_Val[i][2] ** 2) )

    #print(first_pos)
    initial_guess_A = first_pos    # 초기 자석의 위치좌표 값
    initial_guess_H = first_h # 초기 자석의 자계강도 값

    result_pos = leastsq(residuals, initial_guess_A, args=(initial_guess_H))
    #print(result_pos[0])
    first_pos = np.array(result_pos[0])


'''
    for i in range(9):  # 9번 반복하면서 i번째 센서 값 출력
        print(i+1,
              round((array_Val[i][0]), 4), 
              round((array_Val[i][1]), 4), 
              round((array_Val[i][2]), 4), 
              h_val[i])
'''    

# 자석의 자기밀도를 계산하는 함수 
# A: 자석의 현재 위치좌표, P: 센서의 위치좌표, H: 자석의 자계강도
def cal_B(A, H, P):
    global MU0, MU_R, M_T
    N_t = (MU0 * MU_R * M_T) / (4*(np.pi))              # 상수항 계산 
    b1 = (3 * np.dot(H, P.T) * P) / (np.linalg.norm(P-A) ** 5)    # 첫째 항 계산
    b2 = H / (np.linalg.norm(P-A) ** 3)                           # 둘째 항 계산
    
    final_B = N_t * (b1 - b2)                                       # 최종 자기밀도 값 계산
    return final_B                                                  # 최종 자기밀도 값 반환



# 측정한 자기장 값과 계산한 자기장 값 사이의 차이를 계산하는 함수
def residuals(init_pos, init_h):
    global array_Val, P
    differences = [0,0,0] # 센서 값과 계산 값 사이의 잔차 값을 저장하는 배열변수 선언
    for i in range(9):
        buffer_residual = (array_Val[i] - (cal_B(init_pos, init_h, P[i])))
        #print(buffer_residual)
        #print("----") 
        differences[0] += buffer_residual[0]
        differences[1] += buffer_residual[1]
        differences[2] += buffer_residual[2]

    print(differences)
    return differences






# 메인 함수
def main():

    global array_Val


    rospy.init_node('algorithm_pkg_node', anonymous=True)   # 해당 노드의 기본 설정
    rospy.Subscriber('read', String, seperating_Packet)   # /read를 구독하고 seperating_Packet 함수 호출
    rospy.Subscriber('local_sys', PointCloud, callback1x)   # /local_sys를 구독하고 callback1x 함수 호출
    rospy.Subscriber('Is_offset', String, callback_offset)  # /Is_offset을 구독하고 callback_offset 함수 호출

    rospy.spin()    # node 무한 반복






if __name__ == '__main__':
    main()                  # main문 무한 호출

    
from sensor_msgs.msg import PointCloud