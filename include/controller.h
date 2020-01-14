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

void initInput(void);
ControllerData* initControllerData(WORD key, float down, float up, float *dest);
ControllerData* initControllerEvent(WORD key, void (*downEvent)(void), void (*upEvent)(void));
void initControllerDataCross(ControllerData *events[4], WORD up, WORD left, WORD down, WORD right, float dest[2]);
void updateController(void);
void deinitInput(void);

#endif
