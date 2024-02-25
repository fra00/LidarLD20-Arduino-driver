#include "LidarData.h"


LidarData::LidarData(int speed, float startAngle ,float endAngle,int timestamp) {
  this->speed = speed;
  this->startAngle = startAngle;
  this->endAngle = endAngle;
  this->timestamp = timestamp;
}

