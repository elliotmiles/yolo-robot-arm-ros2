import serial
import time
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# If 10 JointStates have been published to /angles with 5 seconds, send the next JointState to the serial port
# Then delay for 15 seconds
# Repeat

class Comms:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        self.ser_ = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)  # Wait for the serial connection to initialize

    def upload(self, data):
        # first converts all list elements to strings, then joins them into one continuous string separated by commas
        comm = ",".join(map(str, data))
        self.ser_.write(f"{comm}\n".encode())

    def close(self):
        self.ser_.close()
        

class CommsNode(Node):
    def __init__(self):
        super().__init__('comms_node')
        self.comms_ = Comms()
        self.subscription_ = self.create_subscription(
            JointState,
            '/angles',
            self.subscription_callback,
            10
        )
        self.timer_ = self.create_timer(5.0, self.timer_callback)
        self.msg_list_ = []
        self.last_send_time_ = 0.0
        self.cooldown_ = 15.0
    
    def subscription_callback(self, msg):
        self.msg_list_.append(msg)


    def timer_callback(self):
        now_ = time.time()

        # if it's been 15 second since a message was sent, and more than 10 have been received in this 5s period, then send a message

        if len(self.msg_list_) >= 10 and (now_ - self.last_send_time_) >= self.cooldown_: 
            # take the last message in the list
            new_msg_ = self.msg_list_[-1]

            # new list for positions only
            deg_positions = []

            for angle in new_msg_.position:
                # convert to degrees & round to 4dp
                deg_positions.append(round(np.degrees(angle), 4))
                
            self.comms_.upload(deg_positions)
            self.get_logger().info(f'Publishing to serial port:\n{new_msg_.name[0]}: {new_msg_.position[0]}\n{new_msg_.name[1]}: {new_msg_.position[1]}\n{new_msg_.name[2]}: {new_msg_.position[2]}\n{new_msg_.name[3]}: {new_msg_.position[3]}')

            # update the last time a message was sent to now
            self.last_send_time_ = now_

        # clear the list    
        self.msg_list_ = []

 

def main():
    rclpy.init()
    comms_node = CommsNode()
    rclpy.spin(comms_node)
    comms_node.comms_.close()
    comms_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
