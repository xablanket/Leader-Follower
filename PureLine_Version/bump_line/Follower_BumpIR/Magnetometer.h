
#ifndef _MAGNETOMETER_H
#define _MAGNETOMETER_H

#include <Wire.h>
#include <LIS3MDL.h>

#define MAX_AXIS 3

class Magnetometer_c {

  public:

    LIS3MDL mag;

    float readings[ MAX_AXIS ];
    float calibrated[ MAX_AXIS ];
    float cal_min[ MAX_AXIS ];
    float cal_max[ MAX_AXIS ];
    float offset[ MAX_AXIS ];
    float scale[ MAX_AXIS ];
    bool  in_calib = false;

    Magnetometer_c () {
    }

    bool initialise() {

      Wire.begin();

      if ( !mag.init() ) {
        return false;
      } else {
        mag.enableDefault();
        return true;
      }
    }

    void getReadings() {
      mag.read();
      readings[0] = mag.m.x;
      readings[1] = mag.m.y;
      readings[2] = mag.m.z;
    }

    float rawMagnitude(){
      getReadings();
      float x = readings[0];
      float y = readings[1];
      float z = readings[2];
      return sqrtf( (x*x) + (y*y) + (z*z) );
    }

    void beginCalibration(){
      in_calib = true;
      for(int i=0;i<MAX_AXIS;i++){
        cal_min[i] =  1e9;
        cal_max[i] = -1e9;
      }
    }

    void sampleCalibration(){
      if(!in_calib) return;
      getReadings();
      for(int i=0;i<MAX_AXIS;i++){
        if (readings[i] < cal_min[i]) cal_min[i] = readings[i];
        if (readings[i] > cal_max[i]) cal_max[i] = readings[i];
      }
    }

    void endCalibration(){
      in_calib = false;
      for(int i=0;i<MAX_AXIS;i++){
        offset[i] = (cal_max[i] + cal_min[i]) * 0.5f;
        float range = (cal_max[i] - cal_min[i]);
        if (range <= 0.0f) range = 1.0f;
        scale[i] = 2.0f / range;
      }
    }

    void calcCalibrated(){
      getReadings();
      for(int i=0;i<MAX_AXIS;i++){
        calibrated[i] = (readings[i] - offset[i]) * scale[i];
      }
    }

    float magnitude(){
      float x = calibrated[0];
      float y = calibrated[1];
      float z = calibrated[2];
      return sqrtf( (x*x) + (y*y) + (z*z) );
    }

};

#endif
