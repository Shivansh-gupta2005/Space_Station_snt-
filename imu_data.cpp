#include <Wire.h>
#include <MPU9250.h>
#include <Kalman.h>

MPU9250 mpu;
Kalman kalmanYaw, kalmanPitch, kalmanRoll;

// Kalman filter parameters
float Q_angle = 0.001;   // Process noise covariance for angle
float Q_bias = 0.003;    // Process noise covariance for bias
float R_measure = 0.03;  // Measurement noise covariance

// State variables for yaw, pitch, roll
float angleYaw, anglePitch, angleRoll;
float gyroBiasYaw = 0, gyroBiasPitch = 0, gyroBiasRoll = 0;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    delay(2000);

    if (!mpu.setup(0x68)) {
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with connection_check example.");
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

            // Print filtered angles
            Serial.print("Filtered Yaw, Pitch, Roll: ");
            Serial.print(angleYaw, 2);
            Serial.print(", ");
            Serial.print(anglePitch, 2);
            Serial.print(", ");
            Serial.println(angleRoll, 2);

            prev_ms = millis();
        }
    }
}