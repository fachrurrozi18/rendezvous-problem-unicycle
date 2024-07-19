#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import tf

positions = []
orientations = []

speed_ratio = 0.1

def pose_callback_factory(index):
    def pose_callback(msg):
        # Update the robot's position and orientation
        positions[index] = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        orientations[index] = tf.transformations.euler_from_quaternion([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w])[2]  # yaw
    return pose_callback

def publish_to_multiple_topics(num_agents):
    rospy.init_node('multi_topic_publisher', anonymous=True)

    global positions, orientations
    positions = [np.array([0, 0]) for _ in range(num_agents)]
    orientations = [0 for _ in range(num_agents)]

    subscribers = []
    publishers = []
    
    for i in range(1, num_agents + 1):
        # Subscribers for each robot's odometry
        topic = f'/tb3_{i}/odom'
        subscribers.append(rospy.Subscriber(topic, Odometry, pose_callback_factory(i - 1)))

        # Publishers for command velocities
        cmd_vel_topic = f'/tb3_{i}/cmd_vel'
        publishers.append(rospy.Publisher(cmd_vel_topic, Twist, queue_size=10))

    rate = rospy.Rate(10) 
    moves = [Twist() for _ in range(num_agents)]

    while not rospy.is_shutdown():
        z = np.array(positions)
        r = np.array([[np.cos(orientations[i]), np.sin(orientations[i])] for i in range(num_agents)])
        s = np.array([[-np.sin(orientations[i]), np.cos(orientations[i])] for i in range(num_agents)])

        for i in range(num_agents):
            lin_x = 0
            ang_z = 0
            for j in range(num_agents):
                if i != j:
                    lin_x += r[i, :] @ (z[j, :] - z[i, :]).T
                    ang_z += s[i, :] @ (z[j, :] - z[i, :]).T
            moves[i].linear.x = speed_ratio * lin_x
            moves[i].angular.z = speed_ratio * ang_z

        for i in range(num_agents):
            publishers[i].publish(moves[i])
        
        rate.sleep()

if __name__ == '__main__':
    try:
        num_agents = int(input("Enter the number of agents: "))
        publish_to_multiple_topics(num_agents)
    except rospy.ROSInterruptException:
        pass
