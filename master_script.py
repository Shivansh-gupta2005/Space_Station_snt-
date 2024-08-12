#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64,Float64MultiArray

def calculation (msg: Float64):
    

    current_heading_float = float(msg.data)  # Convert message data to float

    Kp = 0.5
    Ki = 0.01
    Kd = 0.1

    
    
    desiredHeading = 0.0

    correction = pidControl(desiredHeading, current_heading_float, Kp, Ki, Kd, 0.01)
    
    targetSpeeds = [0.0] * 4
    baseSpeed = 0.5
    for i in range(4):
        speed_change = correction 
        if  i % 2 == 0:
            targetSpeeds[i] = baseSpeed + speed_change
        else:
            targetSpeeds[i] = baseSpeed - speed_change

    

    cmd=Float64MultiArray()
    cmd.data=targetSpeeds
    pub.publish(cmd)

    


def pidControl(setpoint: float, actual: float, kp: float, ki: float , kd: float, dt: float) -> float:

    # Calculate error term
    error = setpoint - actual

    # Proportional term
    proportional = kp * error

    # Integral term (optional)
    integral = 0.0  # Initialize integral term
    if ki != 0:
        # Use a closure to preserve the integral value between calls
        def update_integral():
            nonlocal integral
            integral += error * dt
        update_integral()  # Call the closure to update integral on first use

    # Derivative term
    error_last_calculation = 0.0  # Initialize error for derivative (outside the loop)
    derivative = (error - error_last_calculation) / dt
    error_last_calculation = error  # Update error for next calculation

    # Combine proportional, integral, and derivative terms
    pid_output = proportional + (ki * integral) + (kd * derivative)

    return pid_output



    
    


if __name__ == '__main__':
    rospy.init_node("targetvelocity")
    sub = rospy.Subscriber("std_msgs", Float64, callback=calculation)
    pub = rospy.Publisher("target_vels",Float64MultiArray,queue_size = 10)
    rospy.loginfo("NODE STARTED")

    rospy.spin()
