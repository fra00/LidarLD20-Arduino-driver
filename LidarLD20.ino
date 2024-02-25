#include <Arduino.h>
#include <vector>

#include "LidarLD20.h"
#include "LidarData.h"
#include "PointData.h"

LidarLD20* lidar = new LidarLD20();

void setup() {
  Serial.begin(115200);
  lidar->setup();
  lidar->stepForPoint=1;
}

void loop() {
  
  //std::vector<LidarData> r1 = lidar->read(30);
  std::vector<LidarData> r = lidar->readRange(10,350);
  for(int i=0;i<r.size();i++){
    LidarData d = r[i];
    Serial.print(d.startAngle);
    Serial.print(" ----- ");
    Serial.println(d.points.size());
    //Serial.println(d.timestamp);
  }
  Serial.println("End angle readed");
  delay(10000);
  // if (dataRead!="") return;
  // for (int i = 0; i < fakePacket.size(); ++i) {
  //     processByteSerial(fakePacket[i]);
  // }
  // return;

  // put your main code here, to run repeatedly:
  
}




