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

