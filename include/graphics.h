/**
* @file graphics.h
* \~english @brief Defines functions to render graphics.
* \~japanese @brief グラフィック系の関数定義。
*/

#ifndef GRAPHICS_H
#define GRAPHICS_H

#include<Windows.h>

#include "./vector.h"
#include "./image.h"

typedef struct {
	float components[4];
	unsigned char color;
} Vertex;

void initScreen(int width, int height);
int* getScreenSize(int size[2]);
void setDivideByZ(int value);
void setFakeZ(int use, float val);
void setZNear(float value);
void setColorFilterAND(unsigned char filter);
void setColorFilterOR(unsigned char filter);
float *initZBuffer(unsigned int width, unsigned int height);
void flushBuffer(Image *image);

Vertex initVertex(float x, float y, float z, unsigned char color);

void pushTransformation(void);
void popTransformation(void);
float (*getTransformation(float out[4][4]))[4];
void clearTransformation(void);
void setCameraMat4(float mat[4][4]);
void clearCameraMat4(void);
void translateTransformation(float dx, float dy, float dz);
void scaleTransformation(float sx, float sy, float sz);
void rotateTransformation(float rx, float ry, float rz);

void fillTriangle(Vertex vertices[3], Image *image, const float uv[3][2], float zBuffer[], Image *output);
void fillPolygons(Vector *vertices, Vector *indices, Image *image, Vector *uv, Vector *uvIndices, float zBuffer[], Image *output);

#endif
