#include<Windows.h>

#include "./include/controller.h"
#include "./include/vector.h"

#define NOF_MAX_EVENTS 10

static HANDLE input;
static INPUT_RECORD inputRecords[NOF_MAX_EVENTS];

void initInput(void) {
  input = GetStdHandle(STD_INPUT_HANDLE);
}

Controller initController(void) {
  Controller controller = { 0 };
  return controller;
}

ControllerData initControllerData(WORD key, float down, float up, float *dest) {
  ControllerData event = { 0 };
  event.key = key;
  event.valueDown = down;
  event.valueUp = up;
  event.dest = dest;
  return event;
}

ControllerData initControllerEvent(WORD key, void (*downEvent)(void), void (*upEvent)(void)) {
  ControllerData event = { 0 };
  event.key = key;
  event.downEvent = downEvent;
  event.upEvent = upEvent;
  return event;
}

void initControllerDataCross(ControllerData events[4], WORD up, WORD left, WORD down, WORD right, float dest[2]) {
  events[0] = initControllerData(up, 1.0F, 0.0F, &dest[1]);
  events[1] = initControllerData(left, -1.0F, 0.0F, &dest[0]);
  events[2] = initControllerData(down, -1.0F, 0.0F, &dest[1]);
  events[3] = initControllerData(right, 1.0F, 0.0F, &dest[0]);
}

void updateController(Controller controller) {
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
        resetIteration(&controller.events);
        controllerEvent = nextData(&controller.events);
        while(controllerEvent) {
          if(keyEvent->wVirtualKeyCode == controllerEvent->key) {
            controllerEvent->state = keyEvent->bKeyDown;
            if(keyEvent->bKeyDown) {
              if(controllerEvent->dest) *controllerEvent->dest = controllerEvent->valueDown;
              if(controllerEvent->downEvent) controllerEvent->downEvent();
            } else {
              if(controllerEvent->dest) *controllerEvent->dest = controllerEvent->valueUp;
              if(controllerEvent->upEvent) controllerEvent->upEvent();
            }
          }
          controllerEvent = nextData(&controller.events);
        }
			  break;
			default: break;
		}
	}
}

void clearController(Controller *controller) {
  ControllerData *data;
  iterf(&controller->events, &data) {
    data->state = FALSE;
    if(data->dest) *data->dest = 0.0F;
  }
}
