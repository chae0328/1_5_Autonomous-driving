#!/usr/bin/env python3
import rospy
import time
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

stop_time = 0
start_time = 0
traffic_flag = False

class Control:
    def __init__(self):
        self.stop_distance = 1.5 # 정지 거리 (미터)
        self.decrease_distance = 3.0  # 라이다 감속 거리 (미터)
        self.control_value_pub = rospy.Publisher('/control_value', Twist, queue_size=10)
        self.velocity = 50.0  # 적절한 속도 값으로 조정
        self.decrease_velocity = 30.0
        self.lidar_data = ''
        self.stopline_data = ''
        self.object_data = ''
        self.angle_data = 0.0
        rospy.Subscriber('/angle', Float32, self.lane_angle_callback, queue_size=5)
        rospy.Subscriber('/scan', LaserScan, self.scan_lidar_distance_callback, queue_size=5)
        rospy.Subscriber('/object', String, self.object_callback, queue_size=5)
        rospy.Subscriber('/crosswalk', String, self.cross_callback, queue_size=5)
        self.main_timer = rospy.Timer(rospy.Duration(0.1), self.main)  # 0.1초마다 main 메소드를 호출
    
    def lane_angle_callback(self,msg):
        self.angle_data = msg.data

    def scan_lidar_distance_callback(self, data):
        front_angle_start = -10
        front_angle_end = 10
        start_index, end_index = self.calculate_front_indices(data.angle_min, data.angle_increment, front_angle_start, front_angle_end)
        front_ranges = data.ranges[start_index:end_index]
        if front_ranges:
            min_distance = min(front_ranges)
        else:
            min_distance = float('inf')  

        if min_distance < self.stop_distance:
            self.lidar_data = 'stop'
        elif min_distance < self.decrease_distance:
            self.lidar_data = 'decrease'
        else:
            self.lidar_data = 'go'

    def calculate_front_indices(self, angle_min, angle_increment, front_angle_start, front_angle_end):
        angle_min_deg = math.degrees(angle_min)
        angle_increment_deg = math.degrees(angle_increment)

        start_index = int((front_angle_start - angle_min_deg) / angle_increment_deg)
        end_index = int((front_angle_end - angle_min_deg) / angle_increment_deg)
        
        return start_index, end_index

    def object_callback(self, msg):
        self.object_data = msg.data
    
    def cross_callback(self, msg):
        self.stopline_data = msg.data

    def main(self, event):
        global stop_time, start_time, traffic_flag
        pub_msg = Twist()

        if stop_time != 0:
            stop_time_difference = time.time() - stop_time
            rospy.loginfo(f"stop_time_difference: {stop_time_difference}")
            if stop_time_difference >= 3:
                if stop_time_difference >= 10:
                    if traffic_flag == True and self.object_data == 'stop':
                        pub_msg.linear.x = 0.0
                        pub_msg.angular.z = self.angle_data 
                        self.control_value_pub.publish(pub_msg)
                        rospy.loginfo("Stopped due to traffic light")
                else:
                    stop_time = 0 # 초기화
                    start_time = time.time()
                    traffic_flag = False
                return
            else:
                pub_msg.linear.x = 0.0
                pub_msg.angular.z = self.angle_data 
                self.control_value_pub.publish(pub_msg)
                rospy.loginfo("Stopping due to stop_time condition")
            return
        
        if start_time != 0:
            start_time_difference = time.time() - start_time
            rospy.loginfo(f"start_time_difference: {start_time_difference}")
            if start_time_difference >= 8:
                start_time = 0  # 초기화
            else:
                pub_msg.linear.x = self.velocity
                pub_msg.angular.z = self.angle_data 
                self.control_value_pub.publish(pub_msg)
                rospy.loginfo("Moving forward after stopline stop")
            return
        
        if self.object_data == 'stop' or self.lidar_data == 'stop':
            pub_msg.linear.x = 0.0
            pub_msg.angular.z = self.angle_data 
            self.control_value_pub.publish(pub_msg)
            rospy.loginfo("Stopping due to object or LiDAR data")
            if self.object_data == 'stop':
                traffic_flag = True
                stop_time = time.time()
            return

        if self.stopline_data == 'stop':
            stop_time = time.time()
            rospy.loginfo(f"Stopping due to stopline at {stop_time}")
            return

        if self.stopline_data == 'decrease' or self.lidar_data == 'decrease':
            pub_msg.linear.x = self.velocity - self.decrease_velocity
            pub_msg.angular.z = 0.0
            self.control_value_pub.publish(pub_msg)
            if self.stopline_data == 'decrease':
                rospy.loginfo("Decreasing speed due to stopline data")
            else:
                rospy.loginfo("Decreasing speed due to LiDAR data")
            return
        
        stop_time = 0
        start_time = 0
        pub_msg.linear.x = self.velocity
        pub_msg.angular.z = self.angle_data 
        self.control_value_pub.publish(pub_msg)
        rospy.loginfo("Publishing normal velocity")

if __name__ == '__main__':
    rospy.init_node('control_manager')
    control = Control()
    rospy.spin()
