#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Float64MultiArray

# Define publishers for each wheel's velocity
pubs = [
    rospy.Publisher('wheel1/velocity', Float64, queue_size=10),
    rospy.Publisher('wheel2/velocity', Float64, queue_size=10),
    rospy.Publisher('wheel3/velocity', Float64, queue_size=10),
    rospy.Publisher('wheel4/velocity', Float64, queue_size=10)
]

def distribute_velocity(msg: Float64MultiArray):
    # Ensure the received message contains exactly 4 target velocities
    if len(msg.data) != 4:
        rospy.logwarn("Received target velocities array does not contain 4 elements")
        return
    
    # Publish the target velocity to each wheel's topic
    for i, target_velocity in enumerate(msg.data):
        vel_msg = Float64()
        vel_msg.data = target_velocity
        pubs[i].publish(vel_msg)

if __name__ == '__main__':
    rospy.init_node("velocity_distributor")

    # Subscriber to the target_vels topic
    rospy.Subscriber("target_vels", Float64MultiArray, distribute_velocity)

    rospy.loginfo("Velocity Distributor Node Started")

    rospy.spin()
