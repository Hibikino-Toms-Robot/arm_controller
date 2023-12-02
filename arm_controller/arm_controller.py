import rclpy  # ROS2のPythonモジュールをインポート
from rclpy.node import Node 

import sys
import numpy as np

sys.path.append("/home/hibikinotoms/hibikino_toms_ws/src/arm_controller/arm_controller")
from ik_solver import InverseKinematicsSolver
from multi_arm import MultiJointArm
from suction_unit import Suction_Unit
from toms_msg.srv import ArmService,EndEffectorService
import time

"""
Arm_service_server
autor : yoshida keisuke 
"""

"""
service通信

[input]


[output] 
harvest success decision
"""


class Arm_Controler(Node):  
    def __init__(self):
        super().__init__('arm_controler') 

        # manipulator_function
        self.solver = InverseKinematicsSolver()
        self.multi_joint_arm = MultiJointArm()
        
 
        #service
        self.srv = self.create_service(ArmService, "arm_service", self.arm_host_server)

    def move2pos(self,target):
        x,y,z,alpha = target.x,target.y,target.z,target.approach_direction
        result = self.solver.solve_ik(x, z, alpha, method="newton_raphson", max_iterations=300)
        self.get_logger().info("逆運動学計算")
        self.get_logger().info(f"{result}")
        if result:
            theta1, theta2, theta3 = result
            self.debug_ik_result(theta1, theta2, theta3)
            self.multi_joint_arm.move2target(y,np.degrees(theta1), np.degrees(theta2), np.degrees(theta3))
            
    def debug_ik_result(self,theta1, theta2, theta3):
        self.get_logger().info("モータ角度") 
        self.get_logger().info("---------------------------")
        self.get_logger().info(f"θ1: {round(np.degrees(theta1))} degrees")
        self.get_logger().info(f"θ2: {round(np.degrees(theta2))} degrees")
        self.get_logger().info(f"θ3: {round(np.degrees(theta3))} degrees")
        self.get_logger().info(f"α: {round(np.degrees(theta1) + np.degrees(theta2) + np.degrees(theta3))} degrees")
        self.get_logger().info("アーム位置")
        self.get_logger().info("---------------------------")
        x,y = self.solver.forward_kinematics(theta1, theta2, theta3)
        self.get_logger().info(f"x : {round(x)} y : {round(y)}")
                
    def arm_host_server(self,request, response):
        if request.task == "init_arm":
            self.multi_joint_arm.init_pos()
            self.get_logger().info("初期化完了")
        else :
            target = request.target
            self.get_logger().info(f"{target.x},{target.y},{target.z},{target.approach_direction}") 
            self.move2pos(target)
            #ホームポジションへ移動
            #self.multi_joint_arm.home_pos()
        response.task_comp = True        
        return response
    
def main():
    rclpy.init() 
    node=Arm_Controler() 
    try :
        rclpy.spin(node) 
    except KeyboardInterrupt :
        print("Ctrl+C has been entered")  
        print("End of program")
    rclpy.shutdown() 

if __name__ == '__main__':
    main()
