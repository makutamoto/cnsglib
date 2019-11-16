#include "./include/cnsg.h"

static LARGE_INTEGER frequency;

void initCNSG(unsigned int width, unsigned int height) {
  initInput();
	initGraphics(width, height);
}

float elapsedTime(LARGE_INTEGER start) {
	LARGE_INTEGER current;
	LARGE_INTEGER elapsed;
  if(frequency.QuadPart == 0) QueryPerformanceFrequency(&frequency);
	QueryPerformanceCounter(&current);
	elapsed.QuadPart = current.QuadPart - start.QuadPart;
	return (float)elapsed.QuadPart / frequency.QuadPart;
}

void gameLoop(unsigned int fps, int (*loop)(float, Image*)) {
	LARGE_INTEGER previousClock;
	float delay = 1.0F / fps;
	QueryPerformanceCounter(&previousClock);
	while(TRUE) {
    Image image;
    image.width = 0;
    image.height = 0;
		float elapsed = min(elapsedTime(previousClock), 1.0F);
		QueryPerformanceCounter(&previousClock);
		if(!loop(elapsed, &image)) break;
    if(!(image.width == 0 || image.height == 0)) {
      setBufferImage(image);
      flushBuffer();
      freeImage(image);
    }
		while(elapsedTime(previousClock) < delay);
	}
}
