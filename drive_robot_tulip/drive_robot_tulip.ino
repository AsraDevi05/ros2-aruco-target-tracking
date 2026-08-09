#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>

const char* ssid     = "zahran";
const char* password = "pengentauya";
const int   udpPort  = 8888;

const unsigned long CMD_TIMEOUT_MS = 1000;
unsigned long lastCmdTime = 0;

const int IN1 = 33;
const int IN2 = 25;
const int IN3 = 22;
const int IN4 = 23;

const int freqPWM     = 1000;
const int resolusiPWM = 8;
const int DEADBAND    = 15;

HardwareSerial IMU(2);
const int IMU_RX_PIN = 16;
const int IMU_TX_PIN = 17;
float roll = 0, pitch = 0, yaw = 0;
uint8_t imuBuffer[8];
int bufIndex = 0;
unsigned long lastImuSendTime = 0;

WiFiUDP udp;
IPAddress clientIP;
uint16_t clientPort = 0;

void setMotor(int pwmKiri, int pwmKanan) {
    if (abs(pwmKiri) < DEADBAND) {
        ledcWrite(IN1, 0);
        ledcWrite(IN2, 0);
    } else if (pwmKiri > 0) {
        ledcWrite(IN1, pwmKiri);
        ledcWrite(IN2, 0);
    } else {
        ledcWrite(IN1, 0);
        ledcWrite(IN2, abs(pwmKiri));
    }

    if (abs(pwmKanan) < DEADBAND) {
        ledcWrite(IN3, 0);
        ledcWrite(IN4, 0);
    } else if (pwmKanan > 0) {
        ledcWrite(IN3, pwmKanan);
        ledcWrite(IN4, 0);
    } else {
        ledcWrite(IN3, 0);
        ledcWrite(IN4, abs(pwmKanan));
    }
}

void stopMotor() {
    ledcWrite(IN1, 0);
    ledcWrite(IN2, 0);
    ledcWrite(IN3, 0);
    ledcWrite(IN4, 0);
}

void cmdVelToMotor(float linear, float angular) {
    float left  = linear - angular * 0.5;
    float right = linear + angular * 0.5;
    left  = constrain(left,  -1.0, 1.0);
    right = constrain(right, -1.0, 1.0);
    int pwmL = (int)(left  * 255);
    int pwmR = (int)(right * 255);
    setMotor(pwmL, pwmR);
}

void parseIMU() {
    while (IMU.available()) {
        uint8_t b = IMU.read();
        if (bufIndex == 0 && b != 0xAA) continue;
        if (bufIndex == 1 && b != 0x55) { bufIndex = 0; continue; }
        imuBuffer[bufIndex++] = b;
        if (bufIndex == 8) {
            bufIndex = 0;
            int16_t r = (imuBuffer[2] << 8) | imuBuffer[3];
            int16_t p = (imuBuffer[4] << 8) | imuBuffer[5];
            int16_t y = (imuBuffer[6] << 8) | imuBuffer[7];
            roll  = r / 100.0f;
            pitch = p / 100.0f;
            yaw   = y / 100.0f;
        }
    }
}

void parseUdpPacket(char* buf, size_t len) {
    StaticJsonDocument<128> doc;
    DeserializationError err = deserializeJson(doc, buf, len);
    if (err) return;
    float lin = doc["linear"]  | 0.0f;
    float ang = doc["angular"] | 0.0f;
    cmdVelToMotor(lin, ang);
    lastCmdTime = millis();
}

void processUdp() {
    int len = udp.parsePacket();
    if (len > 0) {
      Serial.print("UDP received, len=");
      Serial.println(len);  // ← TAMBAH INI
      
        clientIP = udp.remoteIP();
        clientPort = udp.remotePort();
        char buf[128];
        int rlen = udp.read(buf, sizeof(buf) - 1);
        if (rlen > 0) {
            buf[rlen] = '\0';
            Serial.print("Data: ");
            Serial.println(buf);  // ← TAMBAH INI
            parseUdpPacket(buf, rlen);
        }
    }
}

void setup() {
    Serial.begin(115200);
    IMU.begin(115200, SERIAL_8N1, IMU_RX_PIN, IMU_TX_PIN);
    delay(500);
    IMU.write(0xA5);
    IMU.write(0x52);
    delay(100);

    ledcAttach(IN1, freqPWM, resolusiPWM);
    ledcAttach(IN2, freqPWM, resolusiPWM);
    ledcAttach(IN3, freqPWM, resolusiPWM);
    ledcAttach(IN4, freqPWM, resolusiPWM);

    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    Serial.println("WiFi connected!");
    Serial.print("ESP32 IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("UDP port: ");
    Serial.println(udpPort);

    udp.begin(udpPort);
    lastCmdTime = millis();
}

void loop() {
    processUdp();
    parseIMU();

    if (WiFi.status() != WL_CONNECTED) {
        WiFi.disconnect();
        WiFi.begin(ssid, password);
        int retry = 0;
        while (WiFi.status() != WL_CONNECTED && retry < 20) {
            delay(500);
            retry++;
        }
    }

    if (millis() - lastCmdTime > CMD_TIMEOUT_MS) {
        stopMotor();
    }

    if (millis() - lastImuSendTime > 100 && clientPort != 0) {
        StaticJsonDocument<128> doc;
        doc["roll"]  = roll;
        doc["pitch"] = pitch;
        doc["yaw"]   = yaw;
        String msg;
        serializeJson(doc, msg);
        udp.beginPacket(clientIP, clientPort);
        udp.print(msg);
        udp.endPacket();
        lastImuSendTime = millis();
    }
}
