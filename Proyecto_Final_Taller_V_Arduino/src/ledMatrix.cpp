#include "Arduino_LED_Matrix.h"   // Include the LED_Matrix library
#include "frames.h"
#include <Arduino.h>

ArduinoLEDMatrix matrix;

void init_displayframe() {
    matrix.begin();
    matrix.loadFrame(LEDMATRIX_HEART_BIG);
}