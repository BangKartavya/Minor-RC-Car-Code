#include "sensor.h"

float Sensor::ZUPTBuff[ZUPT_WIN][3];
int Sensor::ZuptIdx = 0;
float Sensor::PosX = 0;
float Sensor::PosY = 0;
float Sensor::PosXIMU = 0;
float Sensor::PosYIMU = 0;
float Sensor::VelX = 0;
float Sensor::VelY = 0;

void Sensor::Init() {
    UltraSonic::Init();
    GPS::Init();
    MPU::Init();
    for(int i = 0; i < ZUPT_WIN; i++) {
        ZUPTBuff[i][0] = 0.0f;
        ZUPTBuff[i][1] = 0.0f;
        ZUPTBuff[i][2] = 1.0f;
    }
    ZuptIdx = 0;
    PosX = 0;
    PosY = 0;
    PosXIMU = 0;
    PosYIMU = 0;
    VelX = 0;
    VelY = 0;
}

bool Sensor::DetectZUPTWindow() {
    float mean = 0.0f;
    for(int i = 0; i < ZUPT_WIN; i++) {
        float mag = sqrtf(ZUPTBuff[i][0] * ZUPTBuff[i][0] + ZUPTBuff[i][1] * ZUPTBuff[i][1] + ZUPTBuff[i][2] * ZUPTBuff[i][2]);
        mean += mag;
    }
    mean /= ZUPT_WIN;
    float var = 0.0f;
    for(int i = 0; i < ZUPT_WIN; i++) {
        float mag = sqrtf(ZUPTBuff[i][0] * ZUPTBuff[i][0] + ZUPTBuff[i][1] * ZUPTBuff[i][1] + ZUPTBuff[i][2] * ZUPTBuff[i][2]);
        float d = mag - mean;
        var += d * d;
    }
    var /= ZUPT_WIN;
    return (var < ZUPT_VAR_THRESH && fabsf(mean - 9.80665f) < (ZUPT_MAG_THRESH * 9.80665f));
}

void Sensor::IMUZUPT() {
    VelX = 0.0f;
    VelY = 0.0f;
    PosXIMU = PosX;
}

void Sensor::LatLonToXY(double lat, double lon, float& x, float& y) {
    if(!GPS::OriginSet && GPS::Gps.location.isValid()) {
        x = y = 0.0f;
        return;
    }
    double lat0 = GPS::OriginLat * DEG_TO_RAD;
    x = (lon - GPS::OriginLon) * 111320.0 * cos(lat0);
    y = (lat - GPS::OriginLat) * 110540.0;
}

void Sensor::Update(double dt) {
    const float GRAV = 9.80665f;

    MPU::GetAngle(dt);
    MPU::GetAccValues();

    float ax_body = MPU::Acc.x * GRAV;
    float ay_body = MPU::Acc.y * GRAV;
    float az_body = MPU::Acc.z * GRAV;

    float gyroMag = fabs(MPU::Gyro.pitch) + fabs(MPU::Gyro.yaw) + fabs(MPU::Gyro.roll);

    ZUPTBuff[ZuptIdx][0] = ax_body;
    ZUPTBuff[ZuptIdx][1] = ay_body;
    ZUPTBuff[ZuptIdx][2] = az_body;
    ZuptIdx = (ZuptIdx + 1) % ZUPT_WIN;

    float rollRad = MPU::Gyro.roll * (M_PI / 180.0f);
    float pitchRad = MPU::Gyro.pitch * (M_PI / 180.0f);
    float yawRad = MPU::Gyro.yaw * (M_PI / 180.0f);

    float cr = cosf(rollRad), sr = sinf(rollRad);
    float cp = cosf(pitchRad), sp = sinf(pitchRad);
    float cy = cosf(yawRad), sy = sinf(yawRad);

    float R00 = cy * cp;
    float R01 = cy * sp * sr - sy * cr;
    float R02 = cy * sp * cr + sy * sr;

    float R10 = sy * cp;
    float R11 = sy * sp * sr + cy * cr;
    float R12 = sy * sp * cr - cy * sr;

    float R20 = -sp;
    float R21 = cp * sr;
    float R22 = cp * cr;

    float ax_w = R00 * ax_body + R01 * ay_body + R02 * az_body;
    float ay_w = R10 * ax_body + R11 * ay_body + R12 * az_body;
    float az_w = R20 * ax_body + R21 * ay_body + R22 * az_body;

    az_w -= GRAV;

    bool zupt_acc = DetectZUPTWindow();
    bool zupt_gyro = (gyroMag < 0.03f);

    bool isZUPT = zupt_acc && zupt_gyro;

    static float biasX = 0, biasY = 0;
    if(isZUPT) {
        biasX = 0.999f * biasX + 0.001f * ax_w;
        biasY = 0.999f * biasY + 0.001f * ay_w;
    }

    ax_w -= biasX;
    ay_w -= biasY;

    if(isZUPT) {
        VelX = 0;
        VelY = 0;

        PosXIMU = PosX;
        PosYIMU = PosY;

    } else {
        VelX += ax_w * dt;
        VelY += ay_w * dt;

        VelX = constrain(VelX, -3.0f, 3.0f);
        VelY = constrain(VelY, -3.0f, 3.0f);

        PosXIMU += VelX * dt;
        PosYIMU += VelY * dt;
    }

    GPS::Update();

    if(GPS::Gps.location.isUpdated() && GPS::Gps.location.isValid()) {
        float hdop = GPS::Gps.hdop.hdop();

        if(hdop > 2.5f) {
            return;
        }

        double lat = GPS::Gps.location.lat();
        double lon = GPS::Gps.location.lng();

        if(!GPS::OriginSet) {
            GPS::OriginLat = lat;
            GPS::OriginLon = lon;
            GPS::OriginSet = true;
        }

        float gx, gy;
        LatLonToXY(lat, lon, gx, gy);

        float dx = gx - PosXIMU;
        float dy = gy - PosYIMU;
        float dist = sqrtf(dx * dx + dy * dy);

        if(dist < 3.0f) {
            const float beta = 0.08f;
            PosX = (1 - beta) * PosXIMU + beta * gx;
            PosY = (1 - beta) * PosYIMU + beta * gy;
        } else {
            PosX = PosXIMU;
            PosY = PosYIMU;
        }

        PosXIMU = PosX;
        PosYIMU = PosY;
    }
}
