#ifndef POINTDATA_H
#define POINTDATA_H

class PointData {
  public:
    // Dichiarazione attributi
    float angle;
    int intensity;
    int distance;

    // Costruttore di default
    PointData() {}

    // Costruttore con parametri
    PointData(float angle, int distance ,int intensity);

    // Stampa informazioni del punto
    void print();
};

#endif
