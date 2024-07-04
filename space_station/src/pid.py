#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64, Float64MultiArray
from space_station.srv import SetTarget, SetTargetResponse

# Global variables for PID control
integral = 0.0
error_last_calculation = 0.0
desired_heading = 0.0  # Desired setpoint for heading

def calculation(msg: Float64):
    global integral, error_last_calculation, desired_heading
    
    current_heading_float = float(msg.data)  # Convert message data to float

    Kp = 1.0
    Ki = 0.00
    Kd = 0.0

    # Perform PID control to get correction value
    correction = pidControl(0, current_heading_float, Kp, Ki, Kd, 0.01)
    
    target_speeds = [0.0] * 4
    base_speed = desired_heading
    for i in range(4):
        speed_change = correction 
        if i % 2 == 0:
            target_speeds[i] = base_speed + speed_change
        else:
            target_speeds[i] = base_speed - speed_change

    # Publish target speeds
    cmd = Float64MultiArray()
    cmd.data = target_speeds
    pub.publish(cmd)

def pidControl(setpoint: float, actual: float, kp: float, ki: float, kd: float, dt: float) -> float:
    global integral, error_last_calculation

    # Calculate error term
    error = setpoint - actual

    # Proportional term
    proportional = kp * error

    # Update integral term
    integral += error * dt

    # Derivative term
    derivative = (error - error_last_calculation) / dt
    error_last_calculation = error  # Update error for next calculation

    # Combine proportional, integral, and derivative terms
    pid_output = proportional + (ki * integral) + (kd * derivative)

    return pid_output

def handle_set_target(req):
    global desired_heading
    desired_heading = req.target_vel
    rospy.loginfo(f"Target_Velocity updated to: {desired_heading}")
    return SetTargetResponse(success=True, message="Setpoint updated successfully")

if __name__ == '__main__':
    rospy.init_node("target_velocity")
    
    sub = rospy.Subscriber("yaw", Float64, calculation)
    pub = rospy.Publisher("target_vels", Float64MultiArray, queue_size=10)
    
    service = rospy.Service('set_target', SetTarget, handle_set_target)
    rospy.loginfo("Service 'set_target' started. Waiting for setpoint input...")
    
    rospy.loginfo("Node 'target_velocity' started.")
    rospy.spin()
