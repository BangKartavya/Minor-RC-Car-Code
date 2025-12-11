#include "mpu.h"

Angle MPU::Gyro = {0, 0, 0};
Angle MPU::Offset = {0, 0, 0};
MPU6050 MPU::Mpu(0x68);
Acceleration MPU::Acc = {0, 0, 0};

void MPU::GetAccValues() {
    int16_t ax = 0, ay = 0, az = 0;
    int16_t gx = 0, gy = 0, gz = 0;

    Mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    double ax_g = ax / 16384.0;
    double ay_g = ay / 16384.0;
    double az_g = az / 16384.0;

    Acc = {ax_g, ay_g, -az_g};
}

void MPU::Init() {
    Wire.begin(MPU_SDA, MPU_SCL, 100000); // SDA=19, SCL=22
    delay(100);

    Serial.println("Initializing MPU6050...");

    Mpu.initialize();
    delay(100);

    uint8_t id = Mpu.getDeviceID();
    Serial.print("Device ID: 0x");
    Serial.println(id, HEX);

    if(Mpu.testConnection()) {
        Serial.println("MPU6050 connection successful ✅");
    } else {
        Serial.println("MPU6050 connection failed ❌");
    }
    Offset = {0, 0, 0};
    Acc = {0, 0, 0};
    Gyro = {0, 0, 0};
    CalibrateGyro(500);
}

void MPU::CalibrateGyro(int samples) {
    Serial.println("Calibrating gyro... keep still.");
    long sumX = 0, sumY = 0, sumZ = 0;

    for(int i = 0; i < samples; i++) {
        int16_t ax, ay, az, gx, gy, gz;
        Mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        sumX += gx;
        sumY += gy;
        sumZ += gz;
        delay(5);
    }

    Offset.roll = sumX / (double)samples / 131.0;
    Offset.pitch = sumY / (double)samples / 131.0;
    Offset.yaw = sumZ / (double)samples / 131.0;

    Serial.printf("Offsets -> X: %.3f  Y: %.3f  Z: %.3f\n", Offset.roll, Offset.pitch, Offset.yaw);
}

void MPU::GetAngle(double dt) {
    int16_t ax, ay, az, gx, gy, gz;
    Mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // --- Convert raw values ---
    double accX = ax / 16384.0; // g
    double accY = ay / 16384.0;
    double accZ = az / 16384.0;

    double gyroX = (gx / 131.0) - Offset.roll; // deg/s
    double gyroY = (gy / 131.0) - Offset.pitch;
    double gyroZ = (gz / 131.0) - Offset.yaw;

    // --- Integrate gyro to get angles ---
    Gyro.roll += gyroX * dt;
    Gyro.pitch += gyroY * dt;
    Gyro.yaw += gyroZ * dt;

    // --- Calculate tilt angles from accelerometer ---
    float accRoll = atan2(accY, accZ) * 180 / PI;
    float accPitch = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180 / PI;

    // --- Complementary filter ---
    Gyro.roll = ALPHA * Gyro.roll + (1 - ALPHA) * accRoll;
    Gyro.pitch = ALPHA * Gyro.pitch + (1 - ALPHA) * accPitch;

    // store accArr for compatibility
    Acc = {accX, accY, accZ};
}

void MPU::ResetYaw() {
    Gyro.yaw = 0;
}
