/**
* @file controller.h
* \~english @brief Defines functions to read key inputs.
* \~japanese @brief キー入力取得のための関数定義。
*/

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include<Windows.h>

#include "vector.h"

typedef struct {
  WORD key;
  float valueDown;
  float valueUp;
  float *dest;
  int state;
  void (*downEvent)(void);
  void (*upEvent)(void);
} ControllerData;

#define initControllerData(key, down, up, dest) initControllerDataEx(NULL, key, down, up, dest)
#define initControllerEvent(key, downEvent, upEvent) initControllerEventEx(NULL, key, downEvent, upEvent)
#define initControllerDataCross(events, up, left, down, right, dest) initControllerDataCrossEx(NULL, events, up, left, down, right, dest)

void initInput(void);
ControllerData* initControllerDataEx(Vector *list, WORD key, float down, float up, float *dest);
ControllerData* initControllerEventEx(Vector *list, WORD key, void (*downEvent)(void), void (*upEvent)(void));
void initControllerDataCrossEx(Vector *list, ControllerData *events[4], WORD up, WORD left, WORD down, WORD right, float dest[2]);
void updateController(Vector *list);
void deinitInput(void);

#endif
