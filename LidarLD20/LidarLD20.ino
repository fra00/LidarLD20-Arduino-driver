#include <Arduino.h>
#include <vector>

#include "PointData.h"
#include "LidarData.h"
#include "LidarLD20.h"

LidarLD20* lidar = new LidarLD20();

void setup() {
  Serial.begin(230400); //115200
  lidar->setup();
  lidar->stepForPoint=1;
}

void loop() {
  //std::vector<LidarData> r1 = lidar->read(30);
  //std::vector<LidarData> r = lidar->readRange(10,350);
  std::vector<LidarData> r = lidar->readMs(150);
  for(int i=0;i<r.size();i++){
    LidarData d = r[i];
     Serial.print(d.startAngle);
     Serial.print(" ----- ");
     Serial.println(d.points.size());
    //Serial.println(d.timestamp);
  }
  Serial.println("End angle readed");
  delay(10000); 
}




