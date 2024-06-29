#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jun 21 23:21:27 2024

@author: bob
"""

import numpy as np
import serial
import time

MARKER_BYTE = 0xA5

ser = serial.Serial('/dev/tty.usbmodem2086308E484E1')
frames=[]
ser.write(b'd0\n')
ser.write(b'd1\n')
time.sleep(0.1)
ser.read_all()
t0 = time.monotonic()
for i in range(1000):
    frames.append(ser.read(128))
t1=time.monotonic()
ser.write(b'd0\n')
print(f'{len(frames)/(t1-t0):.3f}')

# ser.close()