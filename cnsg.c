#include<windows.h>

#include "./include/cnsg.h"

static LARGE_INTEGER frequency;
static Image screenImage;
static int initialized;
static int sleep;

static int WINAPI ctrlCHandler(DWORD dwCtrlType) {
  deinitCNSG();
  return TRUE;
}

void initCNSG(int argc, char *argv[], unsigned int width, unsigned int height) {
  initSound(argc, argv);
  initInput();
	initScreen(width, height);
  initColorImages();
  SetConsoleCtrlHandler(ctrlCHandler, TRUE);
  screenImage = initImage(width, height, BLACK, NULL_COLOR);
  initialized = TRUE;
}

void deinitCNSG(void) {
  deinitInput();
  deinitSound();
  initialized = FALSE;
}

float elapsedTime(LARGE_INTEGER start) {
	LARGE_INTEGER current;
	LARGE_INTEGER elapsed;
  if(frequency.QuadPart == 0) QueryPerformanceFrequency(&frequency);
	QueryPerformanceCounter(&current);
	elapsed.QuadPart = current.QuadPart - start.QuadPart;
	return (float)elapsed.QuadPart / frequency.QuadPart;
}

int getSleepFlag(void) {
  return sleep;
}

void gameLoop(unsigned int fps) {
	LARGE_INTEGER previousClock;
	float delay = 1.0F / fps;
	QueryPerformanceCounter(&previousClock);
	while(initialized) {
    float elapsed = elapsedTime(previousClock);
    sleep = FALSE;
    if(elapsed > 1.0F) {
      elapsed = 1.0F;
      sleep = TRUE;
    }
		QueryPerformanceCounter(&previousClock);
    if(drawCurrentScene(&screenImage, elapsed)) {
      updateCurrentController(TRUE);
      updateCurrentScene(elapsed);
    } else {
      updateCurrentController(FALSE);
    }
		flushBuffer(&screenImage);
		while(elapsedTime(previousClock) < delay);
	}
  if(screenImage.width != 0) freeImage(&screenImage);
  deinitSound();
}
