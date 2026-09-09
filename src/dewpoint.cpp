//  ****** calculate dew pounts ********

#include <math.h>

float dewpoint(float temperature, float humidity) {

  float a, b;

  if (temperature >= 0) {
    a = 7.5;
    b = 237.3;
  } else if (temperature < 0) {
    a = 7.6;
    b = 240.7;
  }

  float sdd = 6.1078 * pow(10, (a * temperature) / (b + temperature));

  float dd = sdd * (humidity / 100);

  float v = log10(dd / 6.1078);

  float dewpoint = (b * v) / (a - v);

  return dewpoint;
}