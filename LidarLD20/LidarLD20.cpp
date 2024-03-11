#include <Arduino.h> 
#include <vector>
#include <string>
#include <stdint.h>

#include "PointData.h"
#include "LidarData.h"
#include "LidarLD20.h"

//fixed configuration for lidar ld20
int LIDAR_BAUD = 230400;
uint8_t POINTS_PER_PACK = 12;
uint8_t BYTE_HEADER = 0x54;
uint8_t BYTE_VERLEN = 0x2C;

bool isInRange = false;
int startRangeAngle = -1;
int endRangeAngle = -1;
bool endRange = false;

uint8_t processSerialLastByte = 0;

static const uint8_t CrcTable[256] = {
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25,
    0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07,
    0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5, 0x1f, 0x52, 0x85, 0xc8,
    0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
    0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93,
    0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90,
    0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62, 0x97, 0xda, 0x0d, 0x40,
    0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04,
    0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26,
    0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4, 0x7c, 0x31, 0xe6, 0xab,
    0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
    0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0,
    0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd,
    0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f, 0xca, 0x87, 0x50, 0x1d,
    0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67,
    0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45,
    0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7, 0x5d, 0x10, 0xc7, 0x8a,
    0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
    0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1,
    0x7f, 0x32, 0xe5, 0xa8};

/*
 * Initial Identifier: 1 Byte, fixed value of 0x54
 * VerLen: 1 Byte, fixed value of 0x2C
 * Radar Speed: 2 Bytes, unit is degrees per second
 * Starting Angle: 2 Bytes, unit is 0.01 degrees
 * Data: Variable, each measurement data is 3 Bytes
 * End Angle: 2 Bytes, unit is 0.01 degrees
 * Timestamp: 2 Bytes, unit is milliseconds, counts from 30000
 * CRC Check: Variable, checksum of all previous data
 */
 //47
#define PACKET_SIZE (1 + 1 + 2 + 2 + (POINTS_PER_PACK * 3) + 2 + 2 + 1)
static enum {
  HEADER,
  VER_LEN,
  DATA,
} state = HEADER;

std::vector<LidarData> lidarReaded;

uint8_t CalCRC8(const uint8_t *data, uint16_t data_len) {
  uint8_t crc = 0;
  while (data_len--) {
    crc = CrcTable[(crc ^ *data) & 0xff];
    data++;
  }
  return crc;
}

LidarData LidarLD20::processPacket(const uint8_t *data)
{
  LidarData frame;
  //packet[0] BYTE_HEADER
  //packet[1] BYTE_VERLEN
  uint16_t radarSpeed = data[3] << 8 | data[2];
  if(radarSpeed == 0) {
    Serial.println("speed 0");
    frame.empty = true;
    return frame;
  }
  uint16_t startAngle = data[5] << 8 | data[4];
  uint16_t endAngle = data[PACKET_SIZE - 4] << 8 | data[PACKET_SIZE - 5];
  uint16_t timestamp = data[PACKET_SIZE - 2] << 8 | data[PACKET_SIZE - 3];
 
  frame.speed = radarSpeed;
  frame.startAngle = (startAngle / 100) % 360;
  frame.endAngle = (endAngle / 100) % 360;
  frame.timestamp = timestamp;

  // Serial.print(frame.startAngle);
  // Serial.print("------");
  // Serial.print(frame.endAngle);
  // Serial.println(" debug");
  
  // frame.empty = true;
  // return frame;

  if(frame.endAngle<frame.startAngle) {
    Serial.println(" invalid endangle");
    frame.empty = true;
    return frame;
  }

  if(!isInRange && startRangeAngle>0) {
    if(frame.startAngle<startRangeAngle) {
      isInRange = true;
    } else {
      frame.empty = true;
      return frame;      
    }
  }
  
  double diffRead = (endAngle / 100 - startAngle / 100  + 360) % 360;
  double maxRange = ((double)radarSpeed * POINTS_PER_PACK / 2300 * 1.5);
  if (diffRead > maxRange) {
    Serial.println(" Read angle too large");
    frame.empty = true;
    return frame;
  }

  uint32_t diff = ((uint32_t)endAngle + 36000 - startAngle) % 36000;
  float step = diff / (POINTS_PER_PACK - 1) / 100.0;
  float start = startAngle / 100.0;
   
  // Process lidar points
  for (int i = 0; i < POINTS_PER_PACK ; i = i+stepForPoint)
  {
    uint16_t distance = data[7 + i * 3] << 8 | data[6 + i * 3];
    uint8_t intensity = data[8 + i * 3];
    if (filterIntensity && intensity < filterIntensityValue) continue;
    // if (distance == 0) continue;
    // if (distance > 3000) continue;
    PointData p;
    float angle = start + step * i;
    // // Max angle of 360 degrees
    p.angle = fmod(angle, 360);
    p.distance = distance;
    p.intensity = intensity;
    frame.points.push_back(PointData(p.angle, p.distance, p.intensity));
  }

  if(isInRange && endRangeAngle>0) {
    if(frame.endAngle>endRangeAngle) {
      isInRange = false;
      endRange = true;
    }
  }
  // Serial.println(" parsed");
  return frame;  
}

void LidarLD20::processByteSerial (int incomingByte) {
  static uint16_t count = 0;
  static uint8_t tmp[128] = {0};

  if(processSerialLastByte == 0x54 && incomingByte == 0x2C && count==0) {
      tmp[0] = BYTE_HEADER;
      tmp[1] = BYTE_VERLEN;
      // Serial.print(tmp[0],HEX);
      // Serial.print("|");
      // Serial.print(tmp[1],HEX);
      // Serial.print("|");
      count = 2;
    } else {
      if(count>=2) {
        // Serial.print(incomingByte,HEX);
        // Serial.print("|");
        tmp[count++] = incomingByte;
        if (count >= PACKET_SIZE)  //full packet received
        {
          uint8_t crc = CalCRC8(tmp, PACKET_SIZE - 1);
          if (crc != tmp[PACKET_SIZE - 1]) {
            // Serial.print(" CRC error ");
            // Serial.println(crc,HEX);
            count = 0;
            return;
          }
          LidarData r = processPacket(tmp);
          if(!r.empty) {
            lidarReaded.push_back(r);
          } else {
            Serial.println("empty packet");
          }
          count = 0;
        }
      } else {
        // Serial.print("lost byte");
        // Serial.println(incomingByte,HEX);
      }
    }
    processSerialLastByte = incomingByte;
    return;  
  
}

void LidarLD20::getFromSerial () {
  if (Serial1.available()) {
    //  int byte = Serial1.read();
    //  processByteSerial(byte);
    //  return;

    const int bufferSize = 141; // Dimensione del buffer per i byte in ingresso
    byte incomingBytes[bufferSize]; // Dichiarazione dell'array per memorizzare i byte
    
    int bytesRead = Serial1.readBytes(incomingBytes, bufferSize);
    for (int i = 0; i < bytesRead; ++i) {
      processByteSerial(incomingBytes[i]);
    }
  }
}


std::vector<LidarData> LidarLD20::read(int numPack) {
  lidarReaded.clear();
  processSerialLastByte = 0;
  while(lidarReaded.size()<numPack){
    // Serial.println(lidarReaded.size());
    getFromSerial();
  }
  return lidarReaded;
}

std::vector<LidarData> LidarLD20::readMs(int millisec) {
  lidarReaded.clear(); 
  processSerialLastByte = 0;
  unsigned long millisecond = millis();
  int endMs = millisecond + millisec;//millisecond + seconds*1000;
  while(millis() < endMs){
    getFromSerial();
    //delay(1);
    //Serial.println(millis());
  }
  Serial.print("--------------------------------------------- ");
  Serial.println(millis());
  return lidarReaded;
}

std::vector<LidarData> LidarLD20::readRange(int startAngle,int endAngle) {
  lidarReaded.clear(); 
  processSerialLastByte = 0;
  startRangeAngle = startAngle;
  endRangeAngle = endAngle;
  while(!endRange){
    getFromSerial();
    //delay(1);
  }
  startRangeAngle = -1;
  endRangeAngle = -1;
  endRange = false;
  return lidarReaded;
}


void LidarLD20::setup() {
  Serial1.begin(LIDAR_BAUD);
  delay(3000);
  Serial.println("Setup terminated");
}

