/**
* @file cnsg.h
* \~english @brief Header to load this library.
* \~japanese @brief ライブラリを読み込むためのヘッダ。
*/

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
#include "./manager.h"

#ifndef __BORLANDC__
#pragma comment(lib, "Winmm.lib")
#endif

void initCNSG(int argc, char *argv[], unsigned int width, unsigned int height);
void deinitCNSG(void);
float elapsedTime(LARGE_INTEGER start);
int getSleepFlag(void);
void gameLoop(unsigned int fps);

#endif
