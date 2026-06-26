#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <stdint.h>

typedef struct {
  uint8_t x;
  uint8_t y;
  uint8_t sw;
} joystick_t;


#endif