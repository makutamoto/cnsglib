#include "./include/cnsg.h"

void initCNSG(unsigned int width, unsigned int height) {
  initInput();
	initGraphics(width, height);
}

static float elapsedTime(LARGE_INTEGER start, LARGE_INTEGER frequency) {
	LARGE_INTEGER current;
	LARGE_INTEGER elapsed;
	QueryPerformanceCounter(&current);
	elapsed.QuadPart = current.QuadPart - start.QuadPart;
	return (float)elapsed.QuadPart / frequency.QuadPart;
}

void gameLoop(unsigned int fps, int (*loop)(float, Image*)) {
	LARGE_INTEGER previousClock;
	LARGE_INTEGER frequency;
	float delay = 1.0F / fps;
	QueryPerformanceFrequency(&frequency);
	QueryPerformanceCounter(&previousClock);
	while(TRUE) {
    Image image;
		float elapsed = elapsedTime(previousClock, frequency);
		QueryPerformanceCounter(&previousClock);
		if(!loop(elapsed, &image)) break;
    setBufferImage(image);
    flushBuffer();
		while(elapsedTime(previousClock, frequency) < delay);
	}
}
