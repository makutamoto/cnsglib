#ifndef CNSG_H
#define CNSG_H

#include "./borland.h"
#include "./colors.h"
#include "./controller.h"
#include "./graphics.h"
#include "./matrix.h"
#include "./node.h"
#include "./scene.h"
#include "./vector.h"
#include "./transition.h"
#include "./triangle.h"
#include "./sound.h"

#ifndef __BORLANDC__
#pragma comment(lib, "Winmm.lib")
#endif

void initCNSG(int argc, char *argv[], unsigned int width, unsigned int height);
void deinitCNSG(void);
float elapsedTime(LARGE_INTEGER start);
void gameLoop(unsigned int fps, int (*loop)(float, Image*, int));

#endif
