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
  int previousState;
  void (*downEvent)(void);
  void (*upEvent)(void);
} ControllerData;

typedef struct {
  ControllerData *data[4];
  float states[2];
} ControllerDataCross;

typedef struct {
  Vector *list;
  int update;
} ControllerListData;

void initInput(void);
ControllerData* initControllerData(Vector *list, WORD key);
ControllerData* initControllerEvent(Vector *list, WORD key, void (*downEvent)(void), void (*upEvent)(void));
ControllerDataCross* initControllerDataCross(Vector *list, WORD up, WORD left, WORD down, WORD right);
void registerControllerList(Vector *list, int update);
void updateController(void);
void deinitInput(void);

#endif
