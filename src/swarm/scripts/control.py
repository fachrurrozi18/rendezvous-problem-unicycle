#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import tf

# Global variables to store the positions and orientations
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

    rate = rospy.Rate(10) # 10 Hz
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


# #!/usr/bin/env python3

# import rospy
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
# import numpy as np
# import tf

# # Global variables to store the positions and orientations
# positions = [np.array([0, 0]), np.array([0, 0]), np.array([0, 0])]
# orientations = [0, 0, 0]

# def pose_callback_1(msg):
#     # Update robot 1's position and orientation
#     positions[0] = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
#     orientations[0] = tf.transformations.euler_from_quaternion([
#         msg.pose.pose.orientation.x,
#         msg.pose.pose.orientation.y,
#         msg.pose.pose.orientation.z,
#         msg.pose.pose.orientation.w])[2]  # yaw

# def pose_callback_2(msg):
#     # Update robot 2's position and orientation
#     positions[1] = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
#     orientations[1] = tf.transformations.euler_from_quaternion([
#         msg.pose.pose.orientation.x,
#         msg.pose.pose.orientation.y,
#         msg.pose.pose.orientation.z,
#         msg.pose.pose.orientation.w])[2]

# def pose_callback_3(msg):
#     # Update robot 3's position and orientation
#     positions[2] = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
#     orientations[2] = tf.transformations.euler_from_quaternion([
#         msg.pose.pose.orientation.x,
#         msg.pose.pose.orientation.y,
#         msg.pose.pose.orientation.z,
#         msg.pose.pose.orientation.w])[2]

# def publish_to_multiple_topics():
#     rospy.init_node('multi_topic_publisher', anonymous=True)

#     # Subscribers for each robot's odometry
#     rospy.Subscriber('/tb3_1/odom', Odometry, pose_callback_1)
#     rospy.Subscriber('/tb3_2/odom', Odometry, pose_callback_2)
#     rospy.Subscriber('/tb3_3/odom', Odometry, pose_callback_3)

#     # Publishers for command velocities
#     pub_cmd_vel = rospy.Publisher('/tb3_1/cmd_vel', Twist, queue_size=10) #agent 1
#     pub_tb3_1 = rospy.Publisher('/tb3_2/cmd_vel', Twist, queue_size=10) #agent 2
#     pub_tb3_2 = rospy.Publisher('/tb3_3/cmd_vel', Twist, queue_size=10) #agent 3

#     rate = rospy.Rate(10) # 10 Hz
#     r1_move = Twist()
#     r2_move = Twist()
#     r3_move = Twist()

#     while not rospy.is_shutdown():
#         # Use global variables for positions and orientations
#         x1, y1 = positions[0]
#         x2, y2 = positions[1]
#         x3, y3 = positions[2]
#         theta1, theta2, theta3 = orientations

#         z = np.array([[x1, y1], [x2, y2], [x3, y3]])
#         r = np.array([[np.cos(theta1), np.sin(theta1)],
#                       [np.cos(theta2), np.sin(theta2)],
#                       [np.cos(theta3), np.sin(theta3)]])
#         s = np.array([[-np.sin(theta1), np.cos(theta1)],
#                       [-np.sin(theta2), np.cos(theta2)],
#                       [-np.sin(theta3), np.cos(theta3)]])

#         # Compute control values
#         r1_move.linear.x = r[0,:] @ (z[1,:]-z[0,:]).T + r[0,:] @ (z[2,:]-z[0,:]).T
#         r1_move.angular.z = s[0,:] @ (z[1,:]-z[0,:]).T + s[0,:] @ (z[2,:]-z[0,:]).T

#         r2_move.linear.x = r[1,:] @ (z[0,:]-z[1,:]).T + r[1,:] @ (z[2,:]-z[1,:]).T
#         r2_move.angular.z = s[1,:] @ (z[0,:]-z[1,:]).T + s[1,:] @ (z[2,:]-z[1,:]).T

#         r3_move.linear.x = r[2,:] @ (z[0,:]-z[2,:]).T + r[2,:] @ (z[1,:]-z[2,:]).T
#         r3_move.angular.z = s[2,:] @ (z[0,:]-z[2,:]).T + s[2,:] @ (z[1,:]-z[2,:]).T

#         # Publish the same command to all topics
#         pub_cmd_vel.publish(r1_move)
#         pub_tb3_1.publish(r2_move)
#         pub_tb3_2.publish(r3_move)
#         rate.sleep()

# if __name__ == '__main__':
#     try:
#         publish_to_multiple_topics()
#     except rospy.ROSInterruptException:
#         pass
