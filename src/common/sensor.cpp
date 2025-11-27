#include "sensor.h"

MPU6050 mpu(0x68);
float accArr[3] = {0, 0, 0};
unsigned long lastGyroTime = 0;

TinyGPSPlus gps;
HardwareSerial GPS_Serial(2);

float gyroXoffset = 0;
float gyroYoffset = 0;
float gyroZoffset = 0;

float posX = 0;
float posY = 0;
float posX_imu = 0;
float posY_imu = 0;
float velX = 0;
float velY = 0;

float dt = 0;
unsigned long lastTime = 0;
Angle old = {0, 0, 0};

String nmeaLine = "";

// ----------------- internal helpers -----------------
static float zuptBuf[ZUPT_WIN][3];
static int zuptIdx = 0;
static bool originSet = false;
static double originLat = 0.0, originLon = 0.0;

void resetYaw() {
    old.yaw = 0;
    lastGyroTime = micros();
}

void sensorInit() {
    Wire.begin(MPU_SDA, MPU_SCL, 100000); // SDA=19, SCL=22
    delay(100);

    Serial.println("Initializing MPU6050...");

    mpu.initialize();
    delay(100);

    uint8_t id = mpu.getDeviceID();
    Serial.print("Device ID: 0x");
    Serial.println(id, HEX);

    if(mpu.testConnection()) {
        Serial.println("MPU6050 connection successful ✅");
    } else {
        Serial.println("MPU6050 connection failed ❌");
    }

    pinMode(RIGHT_US_TRIG, OUTPUT);
    pinMode(RIGHT_US_ECHO, INPUT);

    pinMode(LEFT_US_TRIG, OUTPUT);
    pinMode(LEFT_US_ECHO, INPUT);

    pinMode(FRONT_US_TRIG, OUTPUT);
    pinMode(FRONT_US_ECHO, INPUT);

    digitalWrite(FRONT_US_TRIG, LOW);
    digitalWrite(LEFT_US_TRIG, LOW);
    digitalWrite(RIGHT_US_TRIG, LOW);

    calibrateGyro(500);
    Serial.println("Starting GPS");
    GPS_Serial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX); // RX=33, TX=32

    delay(500);
    GPS_Serial.println("$PCAS03,1,1,1,1,1,1*00");
    GPS_Serial.println("$PCAS03,1,1,1,1,1,1*4B");
    GPS_Serial.println("$PCAS03,1,1,1,1,1,1*55");
    GPS_Serial.println("$PCAS03,1,1,1,1,1,1*7A");

    if(GPS_Serial.available()) {
        gps.encode(GPS_Serial.read());
        Serial.println("GPS Serial: Data detected immediately after initialization. Assuming connected.");
    } else {
        Serial.println("GPS Serial: No data available immediately. Will check in loop().");
    }

    // init ZUPT buffer with 1g values (rough)
    for(int i = 0; i < ZUPT_WIN; i++) {
        zuptBuf[i][0] = 0.0f;
        zuptBuf[i][1] = 0.0f;
        zuptBuf[i][2] = 1.0f;
    }
    zuptIdx = 0;

    lastTime = micros();
}

double getDistance(int TRIG, int ECHO) {
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);

    int pulseDuration = pulseIn(ECHO, HIGH, 50000);
    double distance = (0.0343 * pulseDuration) / 2;

    return distance == 0 ? 1e9 : distance;
}

void getAccValues() {
    int16_t ax = 0, ay = 0, az = 0;
    int16_t gx = 0, gy = 0, gz = 0;

    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    float ax_g = ax / 16384.0;
    float ay_g = ay / 16384.0;
    float az_g = az / 16384.0;

    float gx_dps = gx / 131.0;
    float gy_dps = gy / 131.0;
    float gz_dps = gz / 131.0;

    accArr[0] = ax_g;
    accArr[1] = ay_g;
    accArr[2] = az_g;
}

void calibrateGyro(int samples) {
    Serial.println("Calibrating gyro... keep still.");
    long sumX = 0, sumY = 0, sumZ = 0;

    for(int i = 0; i < samples; i++) {
        int16_t ax, ay, az, gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        sumX += gx;
        sumY += gy;
        sumZ += gz;
        delay(5);
    }

    gyroXoffset = sumX / (float)samples / 131.0;
    gyroYoffset = sumY / (float)samples / 131.0;
    gyroZoffset = sumZ / (float)samples / 131.0;

    Serial.printf("Offsets -> X: %.3f  Y: %.3f  Z: %.3f\n", gyroXoffset, gyroYoffset, gyroZoffset);
}

void getAngle() {
    unsigned long now = micros();
    dt = (now - lastTime) / 1e6;
    lastTime = now;

    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // --- Convert raw values ---
    double accX = ax / 16384.0; // g
    double accY = ay / 16384.0;
    double accZ = az / 16384.0;

    double gyroX = (gx / 131.0) - gyroXoffset; // deg/s
    double gyroY = (gy / 131.0) - gyroYoffset;
    double gyroZ = (gz / 131.0) - gyroZoffset;

    // --- Integrate gyro to get angles ---
    old.roll += gyroX * dt;
    old.pitch += gyroY * dt;
    old.yaw += gyroZ * dt;

    // --- Calculate tilt angles from accelerometer ---
    float accRoll = atan2(accY, accZ) * 180 / PI;
    float accPitch = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180 / PI;

    // --- Complementary filter ---
    old.roll = ALPHA * old.roll + (1 - ALPHA) * accRoll;
    old.pitch = ALPHA * old.pitch + (1 - ALPHA) * accPitch;

    // yaw remains integrated gyro (you may optionally add magnetometer fusion later)
    // store accArr for compatibility
    accArr[0] = (float)accX;
    accArr[1] = (float)accY;
    accArr[2] = (float)accZ;
}

static bool detectZUPT_window() {
    // compute magnitude mean and variance from zuptBuf
    float mean = 0.0f;
    for(int i = 0; i < ZUPT_WIN; i++) {
        float mag = sqrtf(zuptBuf[i][0] * zuptBuf[i][0] + zuptBuf[i][1] * zuptBuf[i][1] + zuptBuf[i][2] * zuptBuf[i][2]);
        mean += mag;
    }
    mean /= ZUPT_WIN;
    float var = 0.0f;
    for(int i = 0; i < ZUPT_WIN; i++) {
        float mag = sqrtf(zuptBuf[i][0] * zuptBuf[i][0] + zuptBuf[i][1] * zuptBuf[i][1] + zuptBuf[i][2] * zuptBuf[i][2]);
        float d = mag - mean;
        var += d * d;
    }
    var /= ZUPT_WIN;
    return (var < ZUPT_VAR_THRESH && fabsf(mean - 1.0f) < ZUPT_MAG_THRESH);
}

void imuZUPT() {
    // externally enforce zero velocity and gently reduce biases
    velX = 0.0f;
    velY = 0.0f;
    posX_imu = posX;

    // small bias decay (if you want to implement bias state later, do here)
    // nothing aggressive on purpose
}

// ----------------- Helper: lat/lon -> meters (ENU approx) -----------------
static void latlon_to_xy(double lat, double lon, float& x, float& y) {
    if(!originSet) {
        x = y = 0.0f;
        return;
    }
    double lat0 = originLat * DEG_TO_RAD;
    x = (lon - originLon) * 111320.0 * cos(lat0);
    y = (lat - originLat) * 110540.0;
}

// ----------------- MAIN: sensorUpdate() - call this every loop() -----------------
void sensorUpdate() {
    // compute dt using micros (non-blocking)
    unsigned long now = micros();
    dt = (now - lastTime) / 1e6;
    if(dt <= 0 || dt > 0.5) dt = 0.01; // sanity clamp
    lastTime = now;

    // 1) update angles (this reads MPU again — cheap)
    getAngle();

    // 2) read raw accel/gyro to compute world-frame linear accel
    int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
    mpu.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);

    // convert to SI units
    float ax_g = ax_raw / 16384.0f;
    float ay_g = ay_raw / 16384.0f;
    float az_g = az_raw / 16384.0f;

    // apply mounting correction if needed (uncomment if upside-down)
    // az_g = -az_g;

    float ax_ms = ax_g * 9.80665f;
    float ay_ms = ay_g * 9.80665f;
    float az_ms = az_g * 9.80665f;

    // compute roll/pitch in radians from complementary filter (old.roll/pitch are in degrees)
    float roll_rad = old.roll * (PI / 180.0);
    float pitch_rad = old.pitch * (PI / 180.0);

    // rotate body accel to world frame (simple rotation using roll,pitch; ignoring yaw because gravity is along z)
    // Using the rotation: aw = R_roll * R_pitch * a_body
    float sin_r = sinf(roll_rad), cos_r = cosf(roll_rad);
    float sin_p = sinf(pitch_rad), cos_p = cosf(pitch_rad);

    // world-frame components (approx)
    float awx = ax_ms * cos_p + az_ms * sin_p;
    float awy = ax_ms * sin_r * sin_p + ay_ms * cos_r - az_ms * sin_r * cos_p;
    float awz = -ax_ms * cos_r * sin_p + ay_ms * sin_r + az_ms * cos_r * cos_p;

    // linear acceleration in world (remove gravity)
    float lin_ax = awx;
    float lin_ay = awy;
    float lin_az = awz - 9.80665f;

    // 3) ZUPT buffer update (use accel in g-units)
    zuptBuf[zuptIdx][0] = ax_g;
    zuptBuf[zuptIdx][1] = ay_g;
    zuptBuf[zuptIdx][2] = az_g;
    zuptIdx = (zuptIdx + 1) % ZUPT_WIN;

    bool isZupt = detectZUPT_window();

    if(isZupt) {
        // zero velocities and gently correct a bias by nudging velocity and position
        velX = 0.0f;
        velY = 0.0f;
        // gently pull IMU integrated pos toward fused pos to avoid jump
        posX_imu = posX;
        posY_imu = posY;
        // you can also implement bias estimation here if needed
    } else {
        // integrate (simple Euler)
        velX += lin_ax * dt;
        velY += lin_ay * dt;

        posX_imu += velX * dt;
        posY_imu += velY * dt;
    }

    // 4) GPS bytes parse (non-blocking) and fusion (when GPS updates)
    gpsUpdate();
    if(gps.location.isUpdated() && gps.location.isValid()) {
        // Serial.print("Lat: ");
        // Serial.print(gps.location.lat(), 6);
        // Serial.print("  Lon: ");
        // Serial.println(gps.location.lng(), 6);
        // Serial.print("Satellites: ");
        // Serial.println(gps.satellites.value());
        double lat = gps.location.lat();
        double lon = gps.location.lng();

        if(!originSet) {
            originLat = lat;
            originLon = lon;
            originSet = true;
        }

        float gx, gy;
        latlon_to_xy(lat, lon, gx, gy);

        // complementary fusion: alpha trusts IMU, (1-alpha) trusts GPS
        const float beta = 0.02f; // weight for GPS (tunable)
        posX = (1.0f - beta) * posX_imu + beta * gx;
        posY = (1.0f - beta) * posY_imu + beta * gy;

        // reset IMU integrated pos toward fused result to avoid long-term divergence
    }
}

void gpsUpdate() {
    while(GPS_Serial.available()) {
        char c = GPS_Serial.read();

        if(c == '$') {
            nmeaLine = "$";
        } else if(nmeaLine.length() > 0) {
            if(c == '\n') {
                // Serial.println("NMEA: " + nmeaLine);
                // feed TinyGPS
                for(char& x : nmeaLine)
                    gps.encode(x);
                gps.encode('\n');
                nmeaLine = "";
            } else if(c != '\r') {
                nmeaLine += c;
            }
        }
    }
}
