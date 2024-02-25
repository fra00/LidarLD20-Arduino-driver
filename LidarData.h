#ifndef LIDARDATA_H
#define LIDARDATA_H

#include <vector>
#include "PointData.h"

class LidarData {
  public:
    // Dichiarazione attributi
    int speed;
    float startAngle;
    float endAngle;
    std::vector<PointData> points;
    int timestamp;
    bool empty = false;

    // Costruttore di default
    LidarData() {}

    // Costruttore con parametri
    LidarData(int speed, float startAngle ,float endAngle,int timestamp);

    // Stampa informazioni del punto
    void print();
};

#endif
