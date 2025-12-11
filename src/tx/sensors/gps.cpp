#include "gps.h"

HardwareSerial GPS::GPS_Serial(2);
String GPS::NMEALine = "";
TinyGPSPlus GPS::Gps;
bool GPS::OriginSet = false;
double GPS::OriginLat = 0.0;
double GPS::OriginLon = 0.0;

void GPS::Update() {
    while(GPS_Serial.available()) {
        char c = GPS_Serial.read();

        if(c == '$') {
            NMEALine = "$";
        } else if(NMEALine.length() > 0) {
            if(c == '\n') {
                for(const char& x : NMEALine)
                    Gps.encode(x);
                Gps.encode('\n');
                NMEALine = "";
            } else if(c != '\r') {
                NMEALine += c;
            }
        }
    }
}

void GPS::Init() {
    Serial.println("Starting GPS");
    GPS_Serial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX); // RX=33, TX=32

    delay(500);
    GPS_Serial.println("$PCAS03,1,1,1,1,1,1*00");
    GPS_Serial.println("$PCAS03,1,1,1,1,1,1*4B");
    GPS_Serial.println("$PCAS03,1,1,1,1,1,1*55");
    GPS_Serial.println("$PCAS03,1,1,1,1,1,1*7A");

    if(GPS_Serial.available()) {
        Gps.encode(GPS_Serial.read());
        Serial.println("GPS Serial: Data detected immediately after initialization. Assuming connected.");
    } else {
        Serial.println("GPS Serial: No data available immediately. Will check in loop().");
    }

    NMEALine = "";
    OriginSet = false;
    OriginLat = 0.0;
    OriginLon = 0.0;
}

void GPS::Print() {
    Serial.println(NMEALine);
}
