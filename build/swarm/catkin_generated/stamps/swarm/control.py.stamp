#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from math import sqrt

class RendezvousAlgorithm:
    def __init__(self):
        rospy.init_node('rendezvous_algorithm', anonymous=True)
        
        # Create publishers and subscribers for each robot
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pub_tb3_1 = rospy.Publisher('/tb3_1/cmd_vel', Twist, queue_size=10)
        self.pub_tb3_2 = rospy.Publisher('/tb3_2/cmd_vel', Twist, queue_size=10)
        
        rospy.Subscriber('/tb3_1/pose', PoseStamped, self.tb3_1_pose_callback)
        rospy.Subscriber('/tb3_2/pose', PoseStamped, self.tb3_2_pose_callback)
        
        # Store the current positions of each robot
        self.tb3_1_pose = PoseStamped()
        self.tb3_2_pose = PoseStamped()
        
        # Set the publishing rate
        self.rate = rospy.Rate(10) # 10 Hz
    
    def tb3_1_pose_callback(self, msg):
        self.tb3_1_pose = msg
    
    def tb3_2_pose_callback(self, msg):
        self.tb3_2_pose = msg
    
    def calculate_rendezvous_velocity(self):
        # Calculate the average position of the robots
        avg_x = (self.tb3_1_pose.pose.position.x + self.tb3_2_pose.pose.position.x) / 2
        avg_y = (self.tb3_1_pose.pose.position.y + self.tb3_2_pose.pose.position.y) / 2
        
        # Calculate the distance to the average position for each robot
        dist_tb3_1 = sqrt((self.tb3_1_pose.pose.position.x - avg_x)**2 + (self.tb3_1_pose.pose.position.y - avg_y)**2)
        dist_tb3_2 = sqrt((self.tb3_2_pose.pose.position.x - avg_x)**2 + (self.tb3_2_pose.pose.position.y - avg_y)**2)
        
        # Adjust velocities based on distance to the average position
        cmd_vel_tb3_1 = Twist()
        cmd_vel_tb3_1.linear.x = -0.5 * dist_tb3_1
        cmd_vel_tb3_1.linear.y = -0.5 * dist_tb3_1
        
        cmd_vel_tb3_2 = Twist()
        cmd_vel_tb3_2.linear.x = -0.5 * dist_tb3_2
        cmd_vel_tb3_2.linear.y = -0.5 * dist_tb3_2
        
        return cmd_vel_tb3_1, cmd_vel_tb3_2
    
    def publish_rendezvous_velocity(self):
        cmd_vel_tb3_1, cmd_vel_tb3_2 = self.calculate_rendezvous_velocity()
        
        self.pub_tb3_1.publish(cmd_vel_tb3_1)
        self.pub_tb3_2.publish(cmd_vel_tb3_2)
    
    def run(self):
        while not rospy.is_shutdown():
            self.publish_rendezvous_velocity()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        rendezvous_node = RendezvousAlgorithm()
        rendezvous_node.run()
    except rospy.ROSInterruptException:
        pass
