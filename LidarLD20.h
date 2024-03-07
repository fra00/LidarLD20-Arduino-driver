#ifndef LIDARLD20_H
#define LIDARLD20_H
#include <vector>
#include "LidarData.h"

class LidarLD20 {
  public:
    void setup();
    bool filterIntensity = false;
    int filterIntensityValue = 200;
    //1 = 12/1 point;   2 = 12/2 point;
    //3 = 12/3 point;   4 = 12/4 point
    //6 = 12/6 point;   12 = 12/12 point
    int stepForPoint=1;
    std::vector<LidarData> read(int numPack);
    std::vector<LidarData> readMs(int millisec);
    std::vector<LidarData> readRange(int startAngle,int endangle);
  private:
    void processByteSerial (int incomingByte);
    LidarData processPacket(const uint8_t *data);
    void getFromSerial ();
};

#endif