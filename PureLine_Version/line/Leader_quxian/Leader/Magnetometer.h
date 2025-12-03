#ifndef _MAGNETOMETER_H
#define _MAGNETOMETER_H

#include <Wire.h>
#include <LIS3MDL.h>

#define MAX_AXIS 3

class Magnetometer_c {

  public:

    LIS3MDL mag;

    float readings[ MAX_AXIS ];

    Magnetometer_c () {
    }

    bool initialise() {

      Wire.begin();

      if ( !mag.init() ) {
        return false;
      } else {
        return true;
      }
    }

    void getReadings() {
      mag.read();
      readings[0] = mag.m.x;
      readings[1] = mag.m.y;
      readings[2] = mag.m.z;
    }

};

#endif
