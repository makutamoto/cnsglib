#ifndef CONTROLLER_H
#define CONTROLLER_H

#include<Windows.h>

#include "vector.h"

typedef struct {
  Vector events;
} Controller;

typedef struct {
  WORD key;
  float valueDown;
  float valueUp;
  float *dest;
  int state;
  void (*downEvent)(void);
  void (*upEvent)(void);
} ControllerData;

void initInput(void);
Controller initController(void);
ControllerData initControllerData(WORD key, float down, float up, float *dest);
ControllerData initControllerEvent(WORD key, void (*downEvent)(void), void (*upEvent)(void));
void initControllerDataCross(ControllerData events[4], WORD up, WORD left, WORD down, WORD right, float dest[2]);
void updateController(Controller controller);
void clearController(Controller *controller);

#endif
