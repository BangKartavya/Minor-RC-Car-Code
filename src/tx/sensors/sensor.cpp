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
    return (var < ZUPT_VAR_THRESH && fabsf(mean - 1.0f) < ZUPT_MAG_THRESH);
}

void Sensor::IMUZUPT() {
    VelX = 0.0f;
    VelY = 0.0f;
    PosXIMU = PosX;
}

void Sensor::LatLonToXY(double lat, double lon, float& x, float& y) {
    if(!GPS::OriginSet) {
        x = y = 0.0f;
        return;
    }
    double lat0 = GPS::OriginLat * DEG_TO_RAD;
    x = (lon - GPS::OriginLon) * 111320.0 * cos(lat0);
    y = (lat - GPS::OriginLat) * 110540.0;
}

void Sensor::Update(double dt) {
    MPU::GetAngle(dt);
    MPU::GetAccValues();

    float rollRad = MPU::Old.roll * (PI / 180.0);
    float pitchRad = MPU::Old.pitch * (PI / 180.0);

    float axMS = MPU::Acc.x * 9.80665f;
    float ayMS = MPU::Acc.y * 9.80665f;
    float azMS = MPU::Acc.z * 9.80665f;

    float sinRoll = sinf(rollRad);
    float cosRoll = cosf(rollRad);
    float sinPitch = sinf(pitchRad);
    float cosPitch = cosf(pitchRad);

    float awX = axMS * cosPitch + azMS * sinPitch;
    float awY = axMS * sinRoll * sinPitch + ayMS * cosRoll - azMS * sinRoll * cosPitch;
    float awZ = -axMS * cosRoll * sinPitch + ayMS * sinRoll + azMS * cosRoll * cosPitch;

    float linAX = awX;
    float linAY = awY;
    float linAZ = awZ - 9.80665f;

    ZUPTBuff[ZuptIdx][0] = MPU::Acc.x;
    ZUPTBuff[ZuptIdx][1] = MPU::Acc.y;
    ZUPTBuff[ZuptIdx][2] = MPU::Acc.z;
    ZuptIdx = (ZuptIdx + 1) % ZUPT_WIN;

    bool isZupt = DetectZUPTWindow();

    if(isZupt) {
        // zero velocities and gently correct a bias by nudging velocity and position
        VelX = 0.0f;
        VelY = 0.0f;
        // gently pull IMU integrated pos toward fused pos to avoid jump
        PosXIMU = PosX;
        PosYIMU = PosY;
        // you can also implement bias estimation here if needed
    } else {
        // integrate (simple Euler)
        VelX += linAX * dt;
        VelY += linAY * dt;

        PosXIMU += VelX * dt;
        PosYIMU += VelY * dt;
    }

    GPS::Update();

    if(GPS::Gps.location.isUpdated() && GPS::Gps.location.isValid()) {
        // Serial.print("Lat: ");
        // Serial.print(gps.location.lat(), 6);
        // Serial.print("  Lon: ");
        // Serial.println(gps.location.lng(), 6);
        // Serial.print("Satellites: ");
        // Serial.println(gps.satellites.value());
        double lat = GPS::Gps.location.lat();
        double lon = GPS::Gps.location.lng();

        if(!GPS::OriginSet) {
            GPS::OriginLat = lat;
            GPS::OriginLon = lon;
            GPS::OriginSet = true;
        }

        float gx, gy;
        LatLonToXY(lat, lon, gx, gy);

        // complementary fusion: alpha trusts IMU, (1-alpha) trusts GPS
        const float beta = 0.02f; // weight for GPS (tunable)
        PosX = (1.0f - beta) * PosXIMU + beta * gx;
        PosY = (1.0f - beta) * PosYIMU + beta * gy;

        // reset IMU integrated pos toward fused result to avoid long-term divergence
    }
}