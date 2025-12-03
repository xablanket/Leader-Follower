
#ifndef _LINESENSORS_H
#define _LINESENSORS_H

#define NUM_SENSORS 5

const int sensor_pins[ NUM_SENSORS ] = { A11, A0, A2, A3, A4 };

#define EMIT_PIN   11

class LineSensors_c {
  
  public:

    float readings[ NUM_SENSORS ];

    float minimum[ NUM_SENSORS ];
    float maximum[ NUM_SENSORS ];
    float scaling[ NUM_SENSORS ];

    float calibrated[ NUM_SENSORS ];

    LineSensors_c() {
      for (int i = 0; i < NUM_SENSORS; i++) {
        readings[i] = 0.0;
        calibrated[i] = 0.0;
        minimum[i] = 1023.0;
        maximum[i] = 0.0;
        scaling[i] = 1.0;
      }
    }

    void initialiseForADC() {

      pinMode( EMIT_PIN, OUTPUT );
      digitalWrite( EMIT_PIN, HIGH );

      for( int sensor = 0; sensor < NUM_SENSORS; sensor++ ) {
        pinMode( sensor_pins[sensor], INPUT_PULLUP );
      }
      
    }

    void readSensorsADC() {

      initialiseForADC();

      for( int sensor = 0; sensor < NUM_SENSORS; sensor++ ) {
        readings[sensor] = (float)analogRead( sensor_pins[sensor] );
      }

    }
    
    void calcCalibratedADC() {

      readSensorsADC();

      for( int sensor = 0; sensor < NUM_SENSORS; sensor++ ) {
        float range = (maximum[sensor] - minimum[sensor]);
        if (range <= 0.0) range = 1.0;
        scaling[sensor] = 1.0 / range;
        float v = (readings[sensor] - minimum[sensor]) * scaling[sensor];
        calibrated[sensor] = v;
      }
      
    }

    void initialiseForDigital() {
      
      pinMode( EMIT_PIN, OUTPUT );
      digitalWrite( EMIT_PIN, HIGH );

    }

    void readSensorsDigital() {
    }

    bool onLine( float threshold = 0.6 ) {
      calcCalibratedADC();
      for (int i = 0; i < NUM_SENSORS; i++) {
        if ( calibrated[i] >= threshold ) return true;
      }
      return false;
    }

};

#endif
