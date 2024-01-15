#!/usr/bin/env python
# -*- coding: utf-8 -*-

import serial

# 시리얼 포트 설정
ser = serial.Serial('/dev/ttyUSB0', 115200)

# 데이터 읽기
while True:
    if ser.in_waiting:
        data = ser.readline().decode('utf-8').rstrip()
        print(data.decode()[:len(data) - 1])