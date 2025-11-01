#ifndef LIDAR_LIB_H
#define LIDAR_LIB_H

#include <Arduino.h>

typedef struct LidarScanNormalMeasureRaw
{
    unsigned char    quality;
    unsigned short   angle_z_q6;
    unsigned short   dist_mm_q2;
} __attribute__((packed)) LidarScanNormalMeasureRaw;



typedef struct LidarDeviceInfo
{
    unsigned char   Model;
    unsigned short  FirmwareVersion;
    unsigned char   HardwareVersion;
    unsigned char   SerialNum[16];
} __attribute__((packed)) DeviceInfo;

struct LidarResponseHeader {
    uint8_t Protocol0;   
    uint8_t Protocol1;          
    uint8_t SubType_Size[4];   //2 and 30, size bytes are in reverse order
    uint8_t DataType;   
};

#define LIDAR_CMD_GET_DEVICE_INFO        0x50
#define LIDAR_CMD_SCAN                   0x20
#define LIDAR_CMD_STOP                   0x25


void ExecuteCmdNoResponse(HardwareSerial& serial, uint8_t cmd);

bool readData(HardwareSerial& serial,uint8_t* desc, int len) ;

void printDeviceInfo(LidarDeviceInfo& deviceInfo);


template<typename T>
bool ExecuteCmdGetResponse(HardwareSerial& serial, uint8_t cmd, T* out)
{

    if (!serial) return false;

    serial.write(0xA5);
    serial.write(cmd);
    serial.flush();

    LidarResponseHeader*  desc = new LidarResponseHeader();
    if(!readData(serial,(uint8_t*)desc,sizeof(LidarResponseHeader)))
    {
        Serial.println("failed to read descriptor");
    }
   
    // Validate header
    if (desc->Protocol0 != 0xA5 || desc->Protocol1 != 0x5A) 
    {
        Serial.println("protocol incorrect");
        return false;
    }

    int size = (desc->SubType_Size[0] | (desc->SubType_Size[1] << 8) | (desc->SubType_Size[2] << 16) | (desc->SubType_Size[3] << 24))  & 0x3FFFFFFF;
    // Check payload size matches struct
    if (size != sizeof(T))
    {
        Serial.write("size mismatch, desc size:");
        Serial.println(size );
        return false;
    } 

    delete desc;
    if(!readData(serial,(uint8_t*)out,size ))
    {
        Serial.println("failed to read payload response");
    }
    // // Copy payload into struct
    // const uint8_t* payload = stream + sizeof(sl_lidar_descriptor_t);
    // memcpy(&out, payload, sizeof(T));

    return true;
}




#endif