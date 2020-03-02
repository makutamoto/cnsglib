#include<Windows.h>

#include "./include/borland.h"
#include "./include/controller.h"
#include "./include/vector.h"

#define NOF_MAX_EVENTS 10

static HANDLE input;
static INPUT_RECORD inputRecords[NOF_MAX_EVENTS];
static Vector registeredControllerLists;

void initInput(void) {
  input = GetStdHandle(STD_INPUT_HANDLE);
  registeredControllerLists = initVector();
}

ControllerData* initControllerData(Vector *list, WORD key) {
  ControllerData *event = calloc(1, sizeof(ControllerData));
  event->key = key;
  event->valueDown = 1.0F;
  event->valueUp = 0.0F;
  push(list, event);
  return event;
}

ControllerData* initControllerEvent(Vector *list, WORD key, void (*downEvent)(void), void (*upEvent)(void)) {
  ControllerData *event = calloc(1, sizeof(ControllerData));
  event->key = key;
  event->downEvent = downEvent;
  event->upEvent = upEvent;
  push(list, event);
  return event;
}

ControllerDataCross* initControllerDataCross(Vector *list, WORD up, WORD left, WORD down, WORD right) {
  int i;
  float valueList[4] = { 1.0F, -1.0F, -1.0F, 1.0F };
  WORD keyList[4];
  ControllerDataCross *dataCross;
  keyList[0] = up;
  keyList[1] = left;
  keyList[2] = down;
  keyList[3] = right;
  dataCross = calloc(sizeof(ControllerDataCross), 1);
  for(i = 0;i < 4;i++) {
    dataCross->data[i] = initControllerData(list, keyList[i]);
    dataCross->data[i]->dest = &dataCross->states[!(i % 2)];
    dataCross->data[i]->valueDown = valueList[i];
  }
  return dataCross;
}

void registerControllerList(Vector *list, int update) {
  ControllerListData *data;
  data = malloc(sizeof(ControllerListData));
  data->list = list;
  data->update = update;
  push(&registeredControllerLists, data);
}

static void runController(KEY_EVENT_RECORD *keyEvent, ControllerData *controllerEvent, int update) {
  if(keyEvent->wVirtualKeyCode == controllerEvent->key) {
    controllerEvent->state = keyEvent->bKeyDown;
    if(keyEvent->bKeyDown) {
      if(controllerEvent->dest) *controllerEvent->dest = controllerEvent->valueDown;
      if(update && controllerEvent->downEvent && controllerEvent->state != controllerEvent->previousState) controllerEvent->downEvent();
    } else {
      if(controllerEvent->dest) *controllerEvent->dest = controllerEvent->valueUp;
      if(update && controllerEvent->upEvent && controllerEvent->state != controllerEvent->previousState) controllerEvent->upEvent();
    }
    controllerEvent->previousState = controllerEvent->state;
  }
}

void updateController(void) {
  int i;
	DWORD nofEvents;
	KEY_EVENT_RECORD *keyEvent;
  ControllerData *controllerEvent;
  GetNumberOfConsoleInputEvents(input, &nofEvents);
	if(nofEvents == 0) return;
	ReadConsoleInput(input, inputRecords, NOF_MAX_EVENTS, &nofEvents);
	for(i = 0;i < (int)nofEvents;i += 1) {
    ControllerListData *listData;
		switch(inputRecords[i].EventType) {
			case KEY_EVENT:
			  keyEvent = &inputRecords[i].Event.KeyEvent;
        iterf(&registeredControllerLists, &listData) {
          iterf(listData->list, &controllerEvent) runController(keyEvent, controllerEvent, listData->update);
        }
        break;
			default: break;
		}
	}
  clearVector(&registeredControllerLists);
}

void deinitInput(void) {
  clearVector(&registeredControllerLists);
}
