import serial
import time
import rclpy
from rclpy.node import Node  
from toms_msg.srv import EndEffectorService

class Suction_Unit(Node):
    def __init__(self):
        super().__init__('end_effector') 
        # self.end_effector_ser = serial.Serial('/dev/ttyUSB-arduino-end-effector', 115200)
        # time.sleep(2)

        #service
        self.srv = self.create_service(EndEffectorService,"end_effector_service", self.end_effector_server)
             
    def end_effector_server(self,request, response):
        end_effector_ser = serial.Serial('/dev/ttyUSB-arduino-end-effector', 115200)
        time.sleep(2)
        self.get_logger().info("収穫動作開始")
        send_data = "S"+"start"+"E"
        end_effector_ser.write(send_data.encode(encoding='utf-8'))
        end_effector_ser.flush()
        try:
            end_effector_ser.timeout = 10 #(s)
            line = end_effector_ser.readline()
            receive_data = line.strip().decode('UTF-8')    
            print("receive_data:", receive_data)
        except serial.serialutil.SerialTimeoutException:
            print("time_out")
        end_effector_ser.close()
        response.task_done = True     
        return response

# class Suction_Unit(Node):
#     def __init__(self):
#         super().__init__('end_effector') 
#         self.end_effector_ser = serial.Serial('/dev/ttyUSB-arduino-end-effector', 115200)
#         time.sleep(2)

#         #service
#         self.srv = self.create_service(EndEffectorService,"end_effector_service", self.end_effector_server)
             
#     def end_effector_server(self,request, response):
#         self.get_logger().info("収穫動作開始")
#         send_data = "S"+"start"+"E"
#         self.data2arduino(send_data)
#         response.task_done = True     
#         return response

#     def debag(self):
#         send_data = "S"+"start"+"E"
#         self.data2arduino(send_data)
    
#     def data2arduino(self,send_data):
#         self.end_effector_ser.write(send_data.encode(encoding='utf-8'))
#         self.end_effector_ser.flush()
#         try:
#             self.end_effector_ser.timeout = 10 #(s)
#             line = self.end_effector_ser.readline()
#             receive_data = line.strip().decode('UTF-8')    
#             print("receive_data:", receive_data)
#         except serial.serialutil.SerialTimeoutException:
#             print("time_out")

# rclpy.init() 
# node = Suction_Unit()
# node.debag()

def main():
    rclpy.init() 
    node = Suction_Unit()
    try :
        rclpy.spin(node) 
    except KeyboardInterrupt :
        print("Ctrl+C has been entered")  
        print("End of program")
    rclpy.shutdown() 

if __name__ == '__main__':
    main()
