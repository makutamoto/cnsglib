#include "./include/cnsg.h"

static LARGE_INTEGER frequency;

void initCNSG(int argc, char *argv[], unsigned int width, unsigned int height) {
  initSound(argc, argv);
  initInput();
	initGraphics(width, height);
}

void deinitCNSG(void) {
  deinitSound();
  deinitGraphics();
}

float elapsedTime(LARGE_INTEGER start) {
	LARGE_INTEGER current;
	LARGE_INTEGER elapsed;
  if(frequency.QuadPart == 0) QueryPerformanceFrequency(&frequency);
	QueryPerformanceCounter(&current);
	elapsed.QuadPart = current.QuadPart - start.QuadPart;
	return (float)elapsed.QuadPart / frequency.QuadPart;
}

void gameLoop(unsigned int fps, int (*loop)(float, Image*, int)) {
	LARGE_INTEGER previousClock;
	float delay = 1.0F / fps;
	QueryPerformanceCounter(&previousClock);
	while(TRUE) {
    Image image;
		float elapsed = elapsedTime(previousClock);
    int sleep = FALSE;
    image.width = 0;
    image.height = 0;
    if(elapsed > 1.0F) {
      elapsed = 1.0F;
      sleep = TRUE;
    }
		QueryPerformanceCounter(&previousClock);
		if(!loop(elapsed, &image, sleep)) break;
    if(!(image.width == 0 || image.height == 0)) {
      setBufferImage(image);
      flushBuffer();
      freeImage(image);
    }
		while(elapsedTime(previousClock) < delay);
	}
}
