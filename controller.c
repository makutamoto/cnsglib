#include<Windows.h>

#include "./include/controller.h"
#include "./include/vector.h"

#define NOF_MAX_EVENTS 10

static HANDLE input;
static INPUT_RECORD inputRecords[NOF_MAX_EVENTS];
static Vector controllerList;

void initInput(void) {
  input = GetStdHandle(STD_INPUT_HANDLE);
}

ControllerData* initControllerDataEx(Vector *list, WORD key, float down, float up, float *dest) {
  ControllerData *event = calloc(1, sizeof(ControllerData));
  event->key = key;
  event->valueDown = down;
  event->valueUp = up;
  event->dest = dest;
  push(list ? list : &controllerList, event);
  return event;
}

ControllerData* initControllerEventEx(Vector *list, WORD key, void (*downEvent)(void), void (*upEvent)(void)) {
  ControllerData *event = calloc(1, sizeof(ControllerData));
  event->key = key;
  event->downEvent = downEvent;
  event->upEvent = upEvent;
  push(list ? list : &controllerList, event);
  return event;
}

void initControllerDataCrossEx(Vector *list, ControllerData *events[4], WORD up, WORD left, WORD down, WORD right, float dest[2]) {
  events[0] = initControllerDataEx(list, up, 1.0F, 0.0F, &dest[1]);
  events[1] = initControllerDataEx(list, left, -1.0F, 0.0F, &dest[0]);
  events[2] = initControllerDataEx(list, down, -1.0F, 0.0F, &dest[1]);
  events[3] = initControllerDataEx(list, right, 1.0F, 0.0F, &dest[0]);
}

static void runController(KEY_EVENT_RECORD *keyEvent, ControllerData *controllerEvent, int update) {
  if(keyEvent->wVirtualKeyCode == controllerEvent->key) {
    controllerEvent->state = keyEvent->bKeyDown;
    if(keyEvent->bKeyDown) {
      if(controllerEvent->dest) *controllerEvent->dest = controllerEvent->valueDown;
      if(update && controllerEvent->downEvent) controllerEvent->downEvent();
    } else {
      if(controllerEvent->dest) *controllerEvent->dest = controllerEvent->valueUp;
      if(update && controllerEvent->upEvent) controllerEvent->upEvent();
    }
  }
}

void updateController(Vector *list, int update) {
  int i;
	DWORD nofEvents;
	KEY_EVENT_RECORD *keyEvent;
  ControllerData *controllerEvent;
  GetNumberOfConsoleInputEvents(input, &nofEvents);
	if(nofEvents == 0) return;
	ReadConsoleInput(input, inputRecords, NOF_MAX_EVENTS, &nofEvents);
	for(i = 0;i < (int)nofEvents;i += 1) {
		switch(inputRecords[i].EventType) {
			case KEY_EVENT:
			  keyEvent = &inputRecords[i].Event.KeyEvent;
        if(list) iterf(list, &controllerEvent) runController(keyEvent, controllerEvent, update);
        iterf(&controllerList, &controllerEvent) runController(keyEvent, controllerEvent, update);
			  break;
			default: break;
		}
	}
}

void deinitInput(void) {
  freeVector(&controllerList);
}
