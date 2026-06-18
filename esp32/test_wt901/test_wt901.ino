#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "BluetoothSerial.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

struct IMUData
{
    float acc[3];     // x, y, z in gs
    float ang_vel[3]; // x, y, z in deg/s
    float angle[3];   // r, p, y deg
    float temp;       // C
    float voltage;    // V
};

class WT901Parser
{
private:
    uint8_t buffer[11];
    uint8_t index = 0;
    IMUData currentData;

    int16_t combineBytes(uint8_t low, uint8_t high)
    {
        return (int16_t)((high << 8) | low);
    }

    bool validateChecksum()
    {
        uint8_t sum = 0;
        for (int i = 0; i < 10; i++)
            sum += buffer[i];
        return sum == buffer[10];
    }

    void processPacket()
    {
        if (!validateChecksum())
            return;

        uint8_t type = buffer[1];
        switch (type)
        {
        case 0x51: // acc
            currentData.acc[0] = combineBytes(buffer[2], buffer[3]) / 32768.0f * 16.0f;
            currentData.acc[1] = combineBytes(buffer[4], buffer[5]) / 32768.0f * 16.0f;
            currentData.acc[2] = combineBytes(buffer[6], buffer[7]) / 32768.0f * 16.0f;
            currentData.temp = combineBytes(buffer[8], buffer[9]) / 100.0f;
            break;

        case 0x52: // omega (ang vel)
            currentData.ang_vel[0] = combineBytes(buffer[2], buffer[3]) / 32768.0f * 2000.0f;
            currentData.ang_vel[1] = combineBytes(buffer[4], buffer[5]) / 32768.0f * 2000.0f;
            currentData.ang_vel[2] = combineBytes(buffer[6], buffer[7]) / 32768.0f * 2000.0f;
            currentData.voltage = combineBytes(buffer[8], buffer[9]) / 100.0f;
            break;

        case 0x53: // angle
            currentData.angle[0] = combineBytes(buffer[2], buffer[3]) / 32768.0f * 180.0f;
            currentData.angle[1] = combineBytes(buffer[4], buffer[5]) / 32768.0f * 180.0f;
            currentData.angle[2] = combineBytes(buffer[6], buffer[7]) / 32768.0f * 180.0f;
            break;
        }
    }

public:
    void update(uint8_t byte)
    {
        if (index == 0 && byte != 0x55)
            return; // header

        buffer[index++] = byte;

        if (index == 11)
        {
            processPacket();
            index = 0;
        }
    }

    IMUData getData() { return currentData; }
};

void drawAxes(Adafruit_SSD1306 &display, float roll, float pitch, float yaw)
{
    float r = roll * DEG_TO_RAD;
    float p = pitch * DEG_TO_RAD;
    float y = yaw * DEG_TO_RAD;

    // rot angle (321)
    float cr = cos(r), sr = sin(r);
    float cp = cos(p), sp = sin(p);
    float cy = cos(y), sy = sin(y);

    // rot mat
    float R[3][3] = {
        {cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr},
        {sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr},
        {-sp, cp * sr, cp * cr}};

    // axes
    float axes[3][3] = {
        {1, 0, 0}, // X
        {0, 1, 0}, // Y
        {0, 0, 1}  // Z
    };

    int cx = 96; // center (right side of screen)
    int cy_screen = 32;
    int scale = 20;

    for (int i = 0; i < 3; i++)
    {
        float x = R[0][i];
        float y2 = R[1][i];

        int x2 = cx + x * scale;
        int y2_screen = cy_screen - y2 * scale;

        display.drawLine(cx, cy_screen, x2, y2_screen, SSD1306_WHITE);
        // display.fillCircle(x2, y2_screen, 3, SSD1306_WHITE);
        display.setCursor(x2 + 4, y2_screen - 4);
        display.print((i == 0) ? "X" : (i == 1) ? "Y"
                                                : "Z");
    }
}

void drawUI(Adafruit_SSD1306 &display, IMUData data, bool connected)
{
    display.clearDisplay();

    // lhs txt
    display.setCursor(0, 0);
    display.printf("Ax: %.1f\nAy: %.1f\nAz: %.1f\n",
                   data.acc[0], data.acc[1], data.acc[2]);

    display.printf("R: %.1f\nP: %.1f\nY: %.1f\n",
                   data.angle[0], data.angle[1], data.angle[2]);

    display.printf("BT: %s\n", connected ? "C" : "NC");

    // rhs axis
    drawAxes(display, data.angle[0], data.angle[1], data.angle[2]);

    display.display();
}

HardwareSerial IMU(2);
WT901Parser parser;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
BluetoothSerial SerialBT;

static bool wasConnected = false;
static unsigned long lastPrint = 0;
static unsigned long lastDisplay = 0;

#define RXD2 16
#define TXD2 17

#define SDA 21
#define SCL 22

void setup()
{
    Serial.begin(115200);

    if (!SerialBT.begin("ESP32_IMU"))
    {
        Serial.println("An error occurred initializing Bluetooth");
    }
    else
    {
        Serial.println("Bluetooth Initialized. Ready to pair.");
    }

    // SerialBT.begin("ESP32_IMU", false); // disable SSP
    // SerialBT.setPin("123456", 6);
    // SerialBT.begin("ESP32_IMU");

    IMU.begin(9600, SERIAL_8N1, RXD2, TXD2);
    Serial.println("WT901 Parser Initialized");

    Wire.begin(SDA, SCL);

    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
    {
        Serial.println("OLED init failed");
        while (true)
            ;
    }

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    Serial.println("WT901 Parser Initialized");
}

void loop()
{
    while (IMU.available())
    {
        parser.update(IMU.read());
    }

    bool connected = SerialBT.hasClient();
    if (connected != wasConnected)
    {
        if (connected)
        {
            Serial.println("--- BT Connected ---");
        }
        else
        {
            Serial.println("--- BT Disconnected ---");
        }
        wasConnected = connected;
    }

    unsigned long currentMillis = millis();
    IMUData data = parser.getData();

    // serial + bt (10Hz)
    if (currentMillis - lastPrint >= 100)
    {
        lastPrint = currentMillis;

        // Serial.printf("Acc: %.2f %.2f %.2f | Angle: %.2f %.2f %.2f\n",
        //               data.acc[0], data.acc[1], data.acc[2],
        //               data.angle[0], data.angle[1], data.angle[2]);

        Serial.printf("%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                      millis() / 1000.0f,
                      data.acc[0], data.acc[1], data.acc[2],
                      data.angle[0], data.angle[1], data.angle[2]);

        // if connect -> send bt
        if (connected)
        {
            SerialBT.printf("%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                            millis() / 1000.0f,
                            data.acc[0], data.acc[1], data.acc[2],
                            data.angle[0], data.angle[1], data.angle[2]);
        }
    }

    // oled update (4Hz)
    if (currentMillis - lastDisplay >= 250)
    {
        drawUI(display, data, connected);
        lastDisplay = currentMillis;
    }

    // yield to ESP32 background tasks (WiFi/BT)
    yield();
}

// void loop() {
//     while (IMU.available()) {
//         parser.update(IMU.read());
//     }

//     static unsigned long lastPrint = 0;
//     if (millis() - lastPrint > 100) {
//         IMUData data = parser.getData();
//         Serial.printf("Acc: %.2f %.2f %.2f | Angle: %.2f %.2f %.2f\n",
//                       data.acc[0], data.acc[1], data.acc[2],
//                       data.angle[0], data.angle[1], data.angle[2]);

//         drawUI(display, data);

//         SerialBT.printf("T %.4f A %.4f %.4f %.4f | RPY %.4f %.4f %.4f\n",
//                 millis() / 1000.0f,
//                 data.acc[0], data.acc[1], data.acc[2],
//                 data.angle[0], data.angle[1], data.angle[2]);

//         lastPrint = millis();
//     }
// }

// #include <Arduino.h>

// HardwareSerial IMU(2); // Serial2

// #define RXD2 16
// #define TXD2 17

// void setup() {
//   Serial.begin(115200);

//   // WT901 usually starts at 9600
//   IMU.begin(9600, SERIAL_8N1, RXD2, TXD2);

//   Serial.println("WT901 IMU reading started...");
// }

// void loop() {
//   while (IMU.available()) {
//     uint8_t c = IMU.read();

//     // Print raw bytes in HEX for debugging
//     Serial.printf("%02X ", c);
//   }
// }

// #include <Wire.h>

// void setup() {
//   Wire.begin();
//   Serial.begin(115200);
//   while (!Serial);

//   Serial.println("Scanning...");
// }

// void loop() {
//   byte error, address;
//   int nDevices = 0;

//   for (address = 1; address < 127; address++) {
//     Wire.beginTransmission(address);
//     error = Wire.endTransmission();

//     if (error == 0) {
//       Serial.print("I2C device found at 0x");
//       Serial.println(address, HEX);
//       nDevices++;
//     }
//   }

//   if (nDevices == 0)
//     Serial.println("No I2C devices found");

//   delay(3000);
// }