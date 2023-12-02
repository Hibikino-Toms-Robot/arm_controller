import serial
import time

import numpy as np
from math import sin,cos,radians
from decimal import Decimal, ROUND_HALF_UP, ROUND_HALF_EVEN
import time
import math

from math import sin,cos,atan2,acos,sqrt

"""
3リンク水平マニピュレータ制御プログラム
autor : yoshida keisuke
"""


"""
[output] 
マニピュレータへの指令値送信(serial通信)

送信データ
S,z_movement,q_1,q_2,q_3,E(string型)
-----------------------------
header : S(start)
footer : E(End)
z軸の移動量 : z_movement
モータへの指令角(degree) : q_n (nはモータの番号)
"""

class MultiJointArm():
    def __init__(self):
        self.ser = serial.Serial('/dev/ttyUSB-arduino-arm',115200)
        time.sleep(2) 

    def data2arduino(self,send_data):
        self.ser.write(send_data.encode(encoding='utf-8'))
        self.ser.flush()
        try:
            self.ser.timeout = 20 #(s)
            line = self.ser.readline()
            receive_data = line.strip().decode('UTF-8')    
            print("receive_data:", receive_data)
        except serial.serialutil.SerialTimeoutException:
            print("time_out")

    def init_pos(self): 
        send_data = "S"+"init"+"E"
        self.data2arduino(send_data)
    
    def home_pos(self): 
        send_data = "S"+"home"+"E"
        self.data2arduino(send_data)
    
    def move2target(self,target_z,angle1,angle2,angle3):
        send_data = "S"+str(target_z)+","+str(angle1)+","+str(angle2)+","+str(angle3)+"E"
        self.data2arduino(send_data)   

