#include "lidar_lib.h"

bool readData(HardwareSerial& serial,uint8_t* desc, int len) 
{
    int i = 0;
    unsigned long start = millis();
    while (i < len) {
        if (serial.available()) {
            desc[i++] = serial.read();
        }
        if (millis() - start > 500) return false; // timeout
    }
    return true;
}

void ExecuteCmdNoResponse(HardwareSerial& serial, uint8_t cmd)
{
    serial.write(0xA5);
    serial.write(cmd);
    serial.flush();
}


void printDeviceInfo(LidarDeviceInfo& deviceInfo)
{
    Serial.print("Mode: ");
    Serial.println(deviceInfo.Model);
    Serial.print("FirmwareVersion: ");
    Serial.println(deviceInfo.FirmwareVersion);
    Serial.print("HardwareVersion: ");
    Serial.println(deviceInfo.HardwareVersion);
    Serial.print("SerialNum: ");
    for(int i =0;i<16;i++)
    {
    Serial.print(deviceInfo.SerialNum[i]);
    }
    Serial.println();
}