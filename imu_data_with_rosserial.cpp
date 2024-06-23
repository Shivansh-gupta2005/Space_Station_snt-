
#include <Wire.h>
#include <MPU9250.h>
#include <Kalman.h>
#include <ros.h>
#include <std_msgs/Float32.h>
//#define USE_USBCON

MPU9250 mpu;
Kalman kalmanYaw, kalmanPitch, kalmanRoll;

// Kalman filter parameters
const float Q_angle = 0.001;   // Process noise covariance for angle
const float Q_bias = 0.003;    // Process noise covariance for bias
const float R_measure = 0.03;  // Measurement noise covariance

// State variables for yaw, pitch, roll
float angleYaw, anglePitch, angleRoll;

// ROS node handle
ros::NodeHandle nh;  // Reduce buffer sizes


// ROS publishers
std_msgs::Float32 yaw_msg;
std_msgs::Float32 pitch_msg;
std_msgs::Float32 roll_msg;
ros::Publisher yaw_pub("yaw", &yaw_msg);
ros::Publisher pitch_pub("pitch", &pitch_msg);
ros::Publisher roll_pub("roll", &roll_msg);

void setup() {
    Serial.begin(9600);
    Wire.begin();
    delay(2000);
    

    // Set up ROS node handle
    nh.initNode();
    nh.advertise(yaw_pub);
    nh.advertise(pitch_pub);
    nh.advertise(roll_pub);

    // Initialize MPU9250
    if (!mpu.setup(0x68)) {
        while (1) {
            nh.logerror("MPU connection failed. Check your connection.");
            delay(5000);
        }
    }

    // Initialize Kalman filters
    kalmanYaw.setAngle(0);
    kalmanPitch.setAngle(0);
    kalmanRoll.setAngle(0);

    kalmanYaw.setQangle(Q_angle);
    kalmanYaw.setQbias(Q_bias);
    kalmanYaw.setRmeasure(R_measure);

    kalmanPitch.setQangle(Q_angle);
    kalmanPitch.setQbias(Q_bias);
    kalmanPitch.setRmeasure(R_measure);

    kalmanRoll.setQangle(Q_angle);
    kalmanRoll.setQbias(Q_bias);
    kalmanRoll.setRmeasure(R_measure);
//    delay(1000);
}

void loop() {
    if (mpu.update()) {
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + 25) {
            float gyroX = mpu.getGyroX();
            float gyroY = mpu.getGyroY();
            float gyroZ = mpu.getGyroZ();

            // Update Kalman filters
            float dt = 0.025; // Time interval in seconds (25 milliseconds)
            angleYaw = kalmanYaw.getAngle(mpu.getYaw(), gyroX, dt);
            anglePitch = kalmanPitch.getAngle(mpu.getPitch(), gyroY, dt);
            angleRoll = kalmanRoll.getAngle(mpu.getRoll(), gyroZ, dt);

            // Publish filtered angles
            yaw_msg.data = angleYaw;
            pitch_msg.data = anglePitch;
            roll_msg.data = angleRoll;

            yaw_pub.publish(&yaw_msg);
            pitch_pub.publish(&pitch_msg);
            roll_pub.publish(&roll_msg);
            delay(100);
            prev_ms = millis();
        }
    }
    nh.spinOnce();
}