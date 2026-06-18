#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

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

    // rot angle
    float cr = cos(r), sr = sin(r);
    float cp = cos(p), sp = sin(p);
    float cy = cos(y), sy = sin(y);

    // rot mat (321)
    // float R[3][3] = {
    //     {cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr},
    //     {sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr},
    //     {-sp, cp * sr, cp * cr}};

    // rot mat (123)
    float R[3][3] = {
        {cp * cy, -cp * sy, sp},
        {sp * sr * cy + cr * sy, cr * cy - sp * sr * sy, -cp * sr},
        {sr * sy - sp * cr * cy, sp * cr * sy + sr * cy, cp * cr}
    };

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

#define RXD2 16
#define TXD2 17

#define SDA 21
#define SCL 22

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
HardwareSerial IMU(2);
WT901Parser parser;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

static bool wasConnected = false;
static unsigned long lastPrint = 0;
static unsigned long lastDisplay = 0;

// Callback to handle connection/disconnection
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) { deviceConnected = true; };
    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        pServer->getAdvertising()->start(); // Restart advertising so we can reconnect
    }
};

void setup() {
    Serial.begin(115200);
    IMU.begin(9600, SERIAL_8N1, RXD2, TXD2);

    // Initialize BLE
    BLEDevice::init("ESP32_IMU_BLE");
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);
    pCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID,
                        BLECharacteristic::PROPERTY_READ |
                        BLECharacteristic::PROPERTY_NOTIFY
                      );
    pCharacteristic->addDescriptor(new BLE2902());
    pService->start();

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    BLEDevice::startAdvertising();

    // Initialize OLED
    Wire.begin(SDA, SCL);
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        while (true);
    }
    display.setTextColor(SSD1306_WHITE);
    Serial.println("BLE Advertising Started...");
}

void loop() {
    while (IMU.available()) {
        parser.update(IMU.read());
    }

    static unsigned long lastPrint = 0;
    static unsigned long lastDisplay = 0;
    unsigned long currentMillis = millis();
    IMUData data = parser.getData();

    if (currentMillis - lastPrint >= 100) {
        lastPrint = currentMillis;

        char payload[64];
        snprintf(payload, sizeof(payload), "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
                 data.acc[0], data.acc[1], data.acc[2],
                 data.angle[0], data.angle[1], data.angle[2]);

        if (deviceConnected) {
            pCharacteristic->setValue(payload);
            pCharacteristic->notify();
        }
        Serial.println(payload);
    }

    if (currentMillis - lastDisplay >= 250) {
        drawUI(display, data, deviceConnected);
        lastDisplay = currentMillis;
    }
}

// #include <BluetoothSerial.h>

// BluetoothSerial SerialBT;

// void setup() {
//   Serial.begin(115200);

//   if (!SerialBT.begin("ESP32_BT")) {
//     Serial.println("Bluetooth init failed!");
//     while (true);
//   }

//   Serial.println("Bluetooth ready");
// }

// void loop() {
//   if (SerialBT.hasClient()) {
//     SerialBT.println("Hello from ESP32");
//     delay(1000);
//   }
// }

// #include <BluetoothSerial.h>
// #include <Wire.h>
// #include <Adafruit_GFX.h>
// #include <Adafruit_SSD1306.h>
// #include "esp_bt_main.h"
// #include "esp_gap_bt_api.h"

// #define SDA 21
// #define SCL 22

// #define SCREEN_WIDTH 128
// #define SCREEN_HEIGHT 64
// #define OLED_RESET -1

// BluetoothSerial SerialBT;
// Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// String deviceName = "ESP32_BT";
// String pinCode = "1234";

// void showMessage(String line1, String line2 = "")
// {
//     display.clearDisplay();
//     display.setTextSize(1);
//     display.setTextColor(SSD1306_WHITE);

//     display.setCursor(0, 10);
//     display.println(line1);

//     display.setCursor(0, 30);
//     display.println(line2);

//     display.display();
// }

// void setup()
// {
//     Serial.begin(115200);

//     // I2C init
//     Wire.begin(SDA, SCL);

//     // OLED init
//     if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
//     {
//         Serial.println("OLED failed");
//         while (true)
//             ;
//     }

//     showMessage("Starting...");

//     // Bluetooth init
//     SerialBT.begin(deviceName);

//     // Set PIN (pairing code)
//     //   esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_FIXED;
//     //   esp_bt_pin_code_t pin_code;
//     //   pin_code[0] = '1';
//     //   pin_code[1] = '2';
//     //   pin_code[2] = '3';
//     //   pin_code[3] = '4';
//     //   esp_bt_gap_set_pin(pin_type, 4, pin_code);

//     // Enable Secure Simple Pairing (SSP)
//     esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
//     esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO; // display + confirm
//     esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));

//     //   showMessage("BT Ready", "PIN: 1234");
//     showMessage("Pairing...", "Confirm code");

//     delay(2000);
// }

// void loop()
// {
//     bool connected = SerialBT.hasClient();

//     if (connected)
//     {
//         showMessage("Connected!", "Sending data...");

//         String msg = "Hello from ESP32: " + String(millis());
//         SerialBT.println(msg);

//         delay(1000);
//     }
//     else
//     {
//         // showMessage("Waiting...", "Pair PIN: 1234");
//         showMessage("Pairing...", "Confirm code");
//         delay(500);
//     }

//     // Optional: read incoming data
//     if (SerialBT.available())
//     {
//         String incoming = SerialBT.readStringUntil('\n');
//         Serial.println("Received: " + incoming);

//         showMessage("RX:", incoming);
//         delay(1500);
//     }
// }