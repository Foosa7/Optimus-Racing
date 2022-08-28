/*
 * Automatic Driving Line calculator
 *
 * License: MTI
 * Author: Optimus Racing
 */

#define PIN 5
#define NUMPIXELS 10

/*
 * Include Arduino libraries
 */
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <TinyGPSPlus.h>
#include "HardwareSerial.h"

/*
 * Include C standard libraries for calculation
 */
#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <stdbool.h>
#include <math.h>

/*
 * Init Neopixels
 */
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
int delayval = 500; // delay for half a second

/*
 * Init GPS
 */
TinyGPSPlus gps;
HardwareSerial SerialGPS(1);

/*
 * Regular int variables.
 */
int i, final0, final1, final2, final3, final4, final5, final6, value;
int32_t px, py;

/*
 * Define a structure to hold the coordinates data
 * 64-bit integer to utilise all decimal points.
 */
typedef struct
{
  int32_t x0, y0, x1, y1, x2, y2;
} triangle;

void setup()
{
  // pin 16 - connect GPS TX pin, pin 17 - connect GPS RX pin
  Serial.begin(115200);
  SerialGPS.begin(9600, SERIAL_8N1, 16, 17);
}

void loop()
{

  /*
   * Define the triangles.
   */
  triangle a0, a1, a2, a3, a4, a5;

  a0.x0 = 77.541712 * 1000000, a0.y0 = 28.366215 * 1000000, a0.x1 = 77.541994 * 1000000, a0.y1 = 28.365974 * 1000000, a0.x2 = 77.541908 * 1000000, a0.y2 = 28.366264 * 1000000;
  a1.x0 = 77.541713 * 1000000, a1.y0 = 28.366214 * 1000000, a1.x1 = 77.541807 * 1000000, a1.y1 = 28.365921 * 1000000, a1.x2 = 77.541995 * 1000000, a1.y2 = 28.365972 * 1000000;
  a2.x0 = 77.541712 * 1000000, a2.y0 = 28.366214 * 1000000, a2.x1 = 77.541692 * 1000000, a2.y1 = 28.365900 * 1000000, a2.x2 = 77.541808 * 1000000, a2.y2 = 28.365926 * 1000000;
  a3.x0 = 77.541712 * 1000000, a3.y0 = 28.366213 * 1000000, a3.x1 = 77.541604 * 1000000, a3.y1 = 28.366185 * 1000000, a3.x2 = 77.541691 * 1000000, a3.y2 = 28.365899 * 1000000;
  a4.x0 = 77.541358 * 1000000, a4.y0 = 28.366124 * 1000000, a4.x1 = 77.541689 * 1000000, a4.y1 = 28.365897 * 1000000, a4.x2 = 77.541601 * 1000000, a4.y2 = 28.366185 * 1000000;
  a5.x0 = 77.541357 * 1000000, a5.y0 = 28.366123 * 1000000, a5.x1 = 77.541448 * 1000000, a5.y1 = 28.365844 * 1000000, a5.x2 = 77.541690 * 1000000, a5.y2 = 28.365897 * 1000000;

  /*
   * Read the GPS coordinates.
   */
  px = (gps.location.lng(), 6) * 100000;
  py = (gps.location.lat(), 6) * 100000;

  /*
   * Compute the location of the point.
   */
  int64_t A0 = 0.5 * ((-a0.y1 * a0.x2 + a0.y0 * (-a0.x1 + a0.x2) + a0.x0 * (a0.y1 - a0.y2) + a0.x1 * a0.y2));
  int sign0 = copysign(1.0, A0); // bool sign = A < 0 ? -1 : 1;
  int64_t s0 = (a0.y0 * a0.x2 - a0.x0 * a0.y2 + (a0.y2 - a0.y0) * px + (a0.x0 - a0.x2) * py) * sign0;
  int64_t t0 = (a0.x0 * a0.y1 - a0.y0 * a0.x1 + (a0.y0 - a0.y1) * px + (a0.x1 - a0.x0) * py) * sign0;
  final0 = s0 > 0 && t0 > 0 && (s0 + t0) < 2 * A0 * sign0;

  // a1
  int64_t A1 = 0.5 * ((-a1.y1 * a1.x2 + a1.y0 * (-a1.x1 + a1.x2) + a1.x0 * (a1.y1 - a1.y2) + a1.x1 * a1.y2));
  int sign1 = copysign(1.0, A1); // bool sign = A < 0 ? -1 : 1;
  int64_t s1 = (a1.y0 * a1.x2 - a1.x0 * a1.y2 + (a1.y2 - a1.y0) * px + (a1.x0 - a1.x2) * py) * sign1;
  int64_t t1 = (a1.x0 * a1.y1 - a1.y0 * a1.x1 + (a1.y0 - a1.y1) * px + (a1.x1 - a1.x0) * py) * sign1;
  final1 = s1 > 0 && t1 > 0 && (s1 + t1) < 2 * A1 * sign1;

  // a2
  int64_t A2 = 0.5 * ((-a2.y1 * a2.x2 + a2.y0 * (-a2.x1 + a2.x2) + a2.x0 * (a2.y1 - a2.y2) + a2.x1 * a2.y2));
  int sign2 = copysign(1.0, A2); // bool sign = A < 0 ? -1 : 1;
  int64_t s2 = (a2.y0 * a2.x2 - a2.x0 * a2.y2 + (a2.y2 - a2.y0) * px + (a2.x0 - a2.x2) * py) * sign2;
  int64_t t2 = (a2.x0 * a2.y1 - a2.y0 * a2.x1 + (a2.y0 - a2.y1) * px + (a2.x1 - a2.x0) * py) * sign2;
  final2 = s2 > 0 && t2 > 0 && (s2 + t2) < 2 * A2 * sign2;

  // a3
  int64_t A3 = 0.5 * ((-a3.y1 * a3.x2 + a3.y0 * (-a3.x1 + a3.x2) + a3.x0 * (a3.y1 - a3.y2) + a3.x1 * a3.y2));
  int sign3 = copysign(1.0, A3); // bool sign = A < 0 ? -1 : 1;
  int64_t s3 = (a3.y0 * a3.x2 - a3.x0 * a3.y2 + (a3.y2 - a3.y0) * px + (a3.x0 - a3.x2) * py) * sign3;
  int64_t t3 = (a3.x0 * a3.y1 - a3.y0 * a3.x1 + (a3.y0 - a3.y1) * px + (a3.x1 - a3.x0) * py) * sign3;
  final3 = s3 > 0 && t3 > 0 && (s3 + t3) < 2 * A3 * sign3;

  // a4
  int64_t A4 = 0.5 * ((-a4.y1 * a4.x2 + a4.y0 * (-a4.x1 + a4.x2) + a4.x0 * (a4.y1 - a4.y2) + a4.x1 * a4.y2));
  int sign4 = copysign(1.0, A4); // bool sign = A < 0 ? -1 : 1;
  int64_t s4 = (a4.y0 * a4.x2 - a4.x0 * a4.y2 + (a4.y2 - a4.y0) * px + (a4.x0 - a4.x2) * py) * sign4;
  int64_t t4 = (a4.x0 * a4.y1 - a4.y0 * a4.x1 + (a4.y0 - a4.y1) * px + (a4.x1 - a4.x0) * py) * sign4;
  final4 = s4 > 0 && t4 > 0 && (s4 + t4) < 2 * A4 * sign4;

  // a5
  int64_t A5 = 0.5 * ((-a5.y1 * a5.x2 + a5.y0 * (-a5.x1 + a5.x2) + a5.x0 * (a5.y1 - a5.y2) + a5.x1 * a5.y2));
  int sign5 = copysign(1.0, A5); // bool sign = A < 0 ? -1 : 1;
  int64_t s5 = (a5.y0 * a5.x2 - a5.x0 * a5.y2 + (a5.y2 - a5.y0) * px + (a5.x0 - a5.x2) * py) * sign5;
  int64_t t5 = (a5.x0 * a5.y1 - a5.y0 * a5.x1 + (a5.y0 - a5.y1) * px + (a5.x1 - a5.x0) * py) * sign5;
  final5 = s5 > 0 && t5 > 0 && (s5 + t5) < 2 * A5 * sign5;


  /*
   * Display on neopixel.
   */
  if (final0 == 1)
  {
    pixels.setPixelColor(2, pixels.Color(0, 50, 0));
    pixels.setPixelColor(3, pixels.Color(0, 255, 0));
    pixels.setPixelColor(4, pixels.Color(0, 50, 0));
  }

  if (final1 == 1)
  {
    pixels.setPixelColor(5, pixels.Color(0, 50, 0));
    pixels.setPixelColor(6, pixels.Color(0, 255, 0));
    pixels.setPixelColor(7, pixels.Color(0, 50, 0));
  }

  if (final2 == 1)
  {
    pixels.setPixelColor(7, pixels.Color(100, 30, 0));
    pixels.setPixelColor(8, pixels.Color(255, 69, 0));
    pixels.setPixelColor(9, pixels.Color(100, 30, 0));
  }

  if (final3 == 1)
  {
    pixels.setPixelColor(5, pixels.Color(0, 50, 0));
    pixels.setPixelColor(6, pixels.Color(0, 255, 0));
    pixels.setPixelColor(7, pixels.Color(0, 50, 0));
  }

  if (final4 == 1)
  {
    pixels.setPixelColor(0, pixels.Color(50, 0, 0));
    pixels.setPixelColor(1, pixels.Color(255, 0, 0));
    pixels.setPixelColor(2, pixels.Color(50, 0, 0));
  }

  if (final5 == 1)
  {
    pixels.setPixelColor(0, pixels.Color(50, 0, 0));
    pixels.setPixelColor(1, pixels.Color(255, 0, 0));
    pixels.setPixelColor(2, pixels.Color(50, 0, 0));
  }

  else
  {
    pixels.setPixelColor(5, pixels.Color(0, 50, 0));
    pixels.setPixelColor(6, pixels.Color(0, 255, 0));
    pixels.setPixelColor(7, pixels.Color(0, 50, 0));
  }

  
  /*
   * Show on neopixel
   */
  pixels.show();
  delay(10);
}
