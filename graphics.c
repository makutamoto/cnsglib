#include<stdio.h>
#include<float.h>
#include<math.h>
#include<Windows.h>

#ifndef __BORLANDC__
#include<stdint.h>
#endif

#include "./include/borland.h"
#include "./include/matrix.h"
#include "./include/graphics.h"
#include "./include/colors.h"
#include "./include/vector.h"

static HANDLE screen;

static float zNearOver;
static float camera[4][4];
static float transformation[4][4];
static Vector matrixStore;
static unsigned int currentStore = 0;

int aabbClear = TRUE;
static float aabbTemp[3][2];

void initScreen(short width, short height) {
	CONSOLE_CURSOR_INFO info = { 1, FALSE };
	COORD bufferSize;
	#ifndef __BORLANDC__
	CONSOLE_FONT_INFOEX font = { sizeof(CONSOLE_FONT_INFOEX) };
	#endif
	screen = CreateConsoleScreenBuffer(GENERIC_WRITE, 0, NULL, CONSOLE_TEXTMODE_BUFFER, NULL);
	SetConsoleActiveScreenBuffer(screen);
	#ifndef __BORLANDC__
	GetCurrentConsoleFontEx(screen, FALSE, &font);
	font.dwFontSize.X = 1;
	font.dwFontSize.Y = 2;
	SetCurrentConsoleFontEx(screen, FALSE, &font);
	// Specify font family.
	#endif
	bufferSize.X = 2 * width;
	bufferSize.Y = height;
	SetConsoleScreenBufferSize(screen, bufferSize);
	SetConsoleCursorInfo(screen, &info);
	SetPriorityClass(GetCurrentProcess(), HIGH_PRIORITY_CLASS);
}

void setZNear(float value) {
	zNearOver = 1.0F / value;
}

float *initZBuffer(unsigned int width, unsigned int height) {
	unsigned long i;
	unsigned long bufferLength = width * height;
	float *buffer = malloc(sizeof(float) * bufferLength);
	for(i = 0;i < bufferLength;i++) buffer[i] = FLT_MAX;
	return buffer;
}

void flushBuffer(Image *image) {
	DWORD nofWritten;
	static COORD cursor = { 0, 0 };
	unsigned long imageLength = image->width * image->height;
	WORD *data = (WORD*)malloc(2 * imageLength * sizeof(WORD));
	size_t index;
	for(index = 0;index < imageLength;index++) {
		unsigned char color;
		WORD attribute;
		color = image->data[index];
		if(color == 7) {
			color = 8;
		} else if(color == 8) {
			color = 7;
		}
		attribute = (WORD)(BACKGROUND_BLUE - 1 + (((color & 8) | ((color & 1) << 2) | (color & 2) | ((color & 4) >> 2)) << 4));
		data[2 * index] = attribute;
		data[2 * index + 1] = attribute;
	}
	WriteConsoleOutputAttribute(screen, data, 2 * imageLength, cursor, &nofWritten);
	free(data);
}

Vertex initVertex(float x, float y, float z, unsigned char color) {
	Vertex vertex;
	vertex.components[0] = x;
	vertex.components[1] = y;
	vertex.components[2] = z;
	vertex.components[3] = 1.0F;
	vertex.color = color;

	return vertex;
}

void pushTransformation(void) {
	float *store = malloc(16 * sizeof(float));
	memcpy_s(store, sizeof(transformation), transformation, sizeof(transformation));
	push(&matrixStore, store);
	currentStore += 1;
}

void popTransformation(void) {
	float *store;
	currentStore -= 1;
	store = (float*)pop(&matrixStore);
	memcpy_s(transformation, sizeof(transformation), store, sizeof(transformation));
	free(store);
}

float (*getTransformation(float out[4][4]))[4] {
	memcpy_s(out, sizeof(transformation), transformation, sizeof(transformation));
	return out;
}

void clearTransformation(void) {
	genIdentityMat4(transformation);
}

void setCameraMat4(float mat[4][4]) {
	memcpy_s(camera, sizeof(camera), mat, sizeof(camera));
}

void clearCameraMat4(void) {
	genIdentityMat4(camera);
}

void translateTransformation(float dx, float dy, float dz) {
	float temp1[4][4], temp2[4][4];
	memcpy_s(temp2, sizeof(transformation), transformation, sizeof(transformation));
	mulMat4(temp2, genTranslationMat4(dx, dy, dz, temp1), transformation);
}

void scaleTransformation(float sx, float sy, float sz) {
	float temp1[4][4], temp2[4][4];
	memcpy_s(temp2, sizeof(transformation), transformation, sizeof(transformation));
	mulMat4(temp2, genScaleMat4(sx, sy, sz, temp1), transformation);
}

void rotateTransformation(float rx, float ry, float rz) {
	float temp1[4][4], temp2[4][4];
	memcpy_s(temp2, sizeof(transformation), transformation, sizeof(transformation));
	mulMat4(temp2, genRotationMat4(rx, ry, rz, temp1), transformation);
}

void clearAABB(void) {
	aabbClear = TRUE;
}

float (*getAABB(float out[3][2]))[2] {
	out[0][0] = aabbTemp[0][0];
	out[0][1] = aabbTemp[0][1];
	out[1][0] = aabbTemp[1][0];
	out[1][1] = aabbTemp[1][1];
	out[2][0] = aabbTemp[2][0];
	out[2][1] = aabbTemp[2][1];

	return out;
}

static float edgeFunction(float x, float y, const float a[2], const float b[2]) {
	return (a[0] - b[0]) * (y - a[1]) - (a[1] - b[1]) * (x - a[0]);
}

static void projectTriangle(float points[3][4], Image image, const float uv[3][2], unsigned char colors[3], float zBuffer[], Image *output) {
	unsigned int i, y, x;
	int tooFar = 0;
	float transformed[3][4];
	float textures[3][2];
	float vertexColors[3];
	unsigned int maxCoord[2], minCoord[2];
	unsigned int halfWidth, halfHeight;
	float area;
	halfWidth = output->width / 2;
	halfHeight = output->height / 2;
	for(i = 0;i < 3;i++) {
		COPY_ARY(transformed[i], points[i]);
		transformed[i][0] *= transformed[i][3];
		transformed[i][1] *= transformed[i][3];
		transformed[i][2] *= transformed[i][3];
		if(transformed[i][2] > 1.0F) tooFar += 1;
		transformed[i][2] *= transformed[i][3];
		transformed[i][0] = roundf(transformed[i][0] * halfWidth + halfWidth);
		transformed[i][1] = roundf(transformed[i][1] * halfHeight + halfHeight);
	}
	if(tooFar == 3) return;
	maxCoord[0] = (unsigned int)max(min(max(max(transformed[0][0], transformed[1][0]), transformed[2][0]), (int)output->width), 0);
	maxCoord[1] = (unsigned int)max(min(max(max(transformed[0][1], transformed[1][1]), transformed[2][1]), (int)output->height), 0);
	minCoord[0] = (unsigned int)max(min(min(transformed[0][0], transformed[1][0]), transformed[2][0]), 0);
	minCoord[1] = (unsigned int)max(min(min(transformed[0][1], transformed[1][1]), transformed[2][1]), 0);
	area = edgeFunction(transformed[0][0], transformed[0][1], transformed[1], transformed[2]);
	if(area == 0.0F) return;
	if(image.data == NULL) {
		vertexColors[0] = colors[0] * transformed[0][3];
		vertexColors[1] = colors[1] * transformed[1][3];
		vertexColors[2] = colors[2] * transformed[2][3];
	} else {
		textures[0][0] = uv[0][0] * transformed[0][3];
		textures[0][1] = uv[0][1] * transformed[0][3];
		textures[1][0] = uv[1][0] * transformed[1][3];
		textures[1][1] = uv[1][1] * transformed[1][3];
		textures[2][0] = uv[2][0] * transformed[2][3];
		textures[2][1] = uv[2][1] * transformed[2][3];
	}
	for(y = minCoord[1];y < maxCoord[1];y++) {
		for(x = minCoord[0];x < maxCoord[0];x++) {
			float weights[3];
			weights[0] = edgeFunction(x + 0.5F, y + 0.5F, transformed[1], transformed[2]);
			weights[1] = edgeFunction(x + 0.5F, y + 0.5F, transformed[2], transformed[0]);
			weights[2] = edgeFunction(x + 0.5F, y + 0.5F, transformed[0], transformed[1]);
			if(weights[0] >= 0.0F && weights[1] >= 0.0F && weights[2] >= 0.0F) {
				size_t index = (size_t)output->width * y + x;
				float depth, z;
				float dataCoords[2];
				unsigned char color;
				weights[0] /= area;
				weights[1] /= area;
				weights[2] /= area;
				depth = 1.0F / (transformed[0][3] * weights[0] + transformed[1][3] * weights[1] + transformed[2][3] * weights[2]);
				z = depth * (transformed[0][2] * weights[0] + transformed[1][2] * weights[1] + transformed[2][2] * weights[2]);
				if(z <= 1.0F && depth < zBuffer[index]) {
					if(image.data == NULL) {
						color = (unsigned char)roundf(depth * (vertexColors[0] * weights[0] + vertexColors[1] * weights[1] + vertexColors[2] * weights[2]));
						if(color != NULL_COLOR) {
							output->data[index] = color;
							zBuffer[index] = depth;
						}
					} else {
						dataCoords[0] = depth * (textures[0][0] * weights[0] + textures[1][0] * weights[1] + textures[2][0] * weights[2]);
						dataCoords[1] =	depth * (textures[0][1] * weights[0] + textures[1][1] * weights[1] + textures[2][1] * weights[2]);
						color = image.data[image.width * min((unsigned int)(floorf(image.height * dataCoords[1])), image.height - 1) + min((unsigned int)(floorf(image.width * dataCoords[0])), image.width - 1)];
						if(color != image.transparent) {
							output->data[index] = color;
							zBuffer[index] = depth;
						}
					}
				}
			}
		}
	}
}

static BOOL calcIntersectionZ(float pointA[4], float pointB[4], float borderZ, float out[4]) {
	float zeroCheck = pointA[2] - pointB[2];
	float t;
	if(zeroCheck == 0.0F) return FALSE;
	t = (borderZ - pointB[2]) / zeroCheck;
	out[0] = (pointA[0] - pointB[0]) * t + pointB[0];
	out[1] = (pointA[1] - pointB[1]) * t + pointB[1];
	out[2] = borderZ;
	out[3] = zNearOver / 2.0F;
	return TRUE;
}

static void calcUVOnLine(const float pointA[3], const float pointB[3], const float point[3], const float uvA[2], const float uvB[2], float out[2]) {
	float weight = distance3(point, pointB) / distance3(pointA, pointB);
	out[0] = (uvA[0] - uvB[0]) * weight + uvB[0];
	out[1] = (uvA[1] - uvB[1]) * weight + uvB[1];
}

void fillTriangle(Vertex vertices[3], Image image, const float uv[3][2], float zBuffer[], Image *output) {
	int i;
	float transformedTemp[3][4], transformed[3][4];
	float triangle[3][4], triangleUV[3][2];
	int clipped[3], displayed[3];
	int nofClipped = 0, nofDisplayed = 0;
	unsigned char colors[3];
	colors[0] = vertices[0].color;
	colors[1] = vertices[1].color;
	colors[2] = vertices[2].color;
	mulMat4Vec4(transformation, vertices[0].components, transformedTemp[0]);
	mulMat4Vec4(transformation, vertices[1].components, transformedTemp[1]);
	mulMat4Vec4(transformation, vertices[2].components, transformedTemp[2]);
	if(aabbClear) {
		aabbTemp[0][0] = transformedTemp[0][0];
		aabbTemp[0][1] = transformedTemp[0][0];
		aabbTemp[1][0] = transformedTemp[0][1];
		aabbTemp[1][1] = transformedTemp[0][1];
		aabbTemp[2][0] = transformedTemp[0][2];
		aabbTemp[2][1] = transformedTemp[0][2];
		aabbClear = FALSE;
	} else {
		if(aabbTemp[0][0] > transformedTemp[0][0]) aabbTemp[0][0] = transformedTemp[0][0];
		if(aabbTemp[0][1] < transformedTemp[0][0]) aabbTemp[0][1] = transformedTemp[0][0];
		if(aabbTemp[1][0] > transformedTemp[0][1]) aabbTemp[1][0] = transformedTemp[0][1];
		if(aabbTemp[1][1] < transformedTemp[0][1]) aabbTemp[1][1] = transformedTemp[0][1];
		if(aabbTemp[2][0] > transformedTemp[0][2]) aabbTemp[2][0] = transformedTemp[0][2];
		if(aabbTemp[2][1] < transformedTemp[0][2]) aabbTemp[2][1] = transformedTemp[0][2];
 	}
	if(aabbTemp[0][0] > transformedTemp[1][0]) aabbTemp[0][0] = transformedTemp[1][0];
	if(aabbTemp[0][1] < transformedTemp[1][0]) aabbTemp[0][1] = transformedTemp[1][0];
	if(aabbTemp[1][0] > transformedTemp[1][1]) aabbTemp[1][0] = transformedTemp[1][1];
	if(aabbTemp[1][1] < transformedTemp[1][1]) aabbTemp[1][1] = transformedTemp[1][1];
	if(aabbTemp[2][0] > transformedTemp[1][2]) aabbTemp[2][0] = transformedTemp[1][2];
	if(aabbTemp[2][1] < transformedTemp[1][2]) aabbTemp[2][1] = transformedTemp[1][2];
	if(aabbTemp[0][0] > transformedTemp[2][0]) aabbTemp[0][0] = transformedTemp[2][0];
	if(aabbTemp[0][1] < transformedTemp[2][0]) aabbTemp[0][1] = transformedTemp[2][0];
	if(aabbTemp[1][0] > transformedTemp[2][1]) aabbTemp[1][0] = transformedTemp[2][1];
	if(aabbTemp[1][1] < transformedTemp[2][1]) aabbTemp[1][1] = transformedTemp[2][1];
	if(aabbTemp[2][0] > transformedTemp[2][2]) aabbTemp[2][0] = transformedTemp[2][2];
	if(aabbTemp[2][1] < transformedTemp[2][2]) aabbTemp[2][1] = transformedTemp[2][2];
	for(i = 0;i < 3;i++) {
		mulMat4Vec4Proj(camera, transformedTemp[i], transformed[i]);
		if(transformed[i][2] < 0.0F) {
			clipped[nofClipped] = i;
			nofClipped += 1;
		} else {
			displayed[nofDisplayed] = i;
			nofDisplayed += 1;
		}
	}
	switch(nofClipped) {
		case 0:
			projectTriangle(transformed, image, uv, colors, zBuffer, output);
			break;
		case 1:
			COPY_ARY(triangle[0], transformed[0]);
			COPY_ARY(triangle[1], transformed[1]);
			COPY_ARY(triangle[2], transformed[2]);
			calcIntersectionZ(transformed[clipped[0]], transformed[displayed[0]], 0, triangle[clipped[0]]);
			COPY_ARY(triangleUV[displayed[0]], uv[displayed[0]]);
			COPY_ARY(triangleUV[displayed[1]], uv[displayed[1]]);
			calcUVOnLine(transformed[clipped[0]], transformed[displayed[0]], triangle[clipped[0]], uv[clipped[0]], uv[displayed[0]], triangleUV[clipped[0]]);
			projectTriangle(triangle, image, triangleUV, colors, zBuffer, output);
			COPY_ARY(triangle[displayed[0]], triangle[displayed[1]]);
			calcIntersectionZ(transformed[clipped[0]], transformed[displayed[1]], 0, triangle[displayed[1]]);
			COPY_ARY(triangleUV[displayed[0]], uv[displayed[1]]);
			calcUVOnLine(transformed[clipped[0]], transformed[displayed[1]], triangle[displayed[1]], uv[clipped[0]], uv[displayed[1]], triangleUV[displayed[1]]);
			projectTriangle(triangle, image, triangleUV, colors, zBuffer, output);
			break;
		case 2:
			COPY_ARY(triangle[displayed[0]], transformed[displayed[0]]);
			calcIntersectionZ(transformed[clipped[0]], transformed[displayed[0]], 0, triangle[clipped[0]]);
			calcIntersectionZ(transformed[clipped[1]], transformed[displayed[0]], 0, triangle[clipped[1]]);
			COPY_ARY(triangleUV[displayed[0]], uv[displayed[0]]);
			calcUVOnLine(transformed[clipped[0]], transformed[displayed[0]], triangle[clipped[0]], uv[clipped[0]], uv[displayed[0]], triangleUV[clipped[0]]);
			calcUVOnLine(transformed[clipped[1]], transformed[displayed[0]], triangle[clipped[1]], uv[clipped[1]], uv[displayed[0]], triangleUV[clipped[1]]);
			projectTriangle(triangle, image, triangleUV, colors, zBuffer, output);
			break;
	}
}

void fillPolygons(Vector vertices, Vector indices, Image image, Vector uv, Vector uvIndices, float zBuffer[], Image *output) {
	unsigned long i1, i2;
	resetIteration(&indices);
	resetIteration(&uvIndices);
	for(i1 = 0;i1 < indices.length / 3;i1++) {
		Vertex triangle[3];
		float triangleUV[3][2];
		for(i2 = 0;i2 < 3;i2++) {
			unsigned long index;
			index = *(unsigned long*)nextData(&indices);
			triangle[i2] = *(Vertex*)dataAt(&vertices, index);
			if(uv.length != 0) {
				float *uvPointer;
				index = *(unsigned long*)nextData(&uvIndices);
				uvPointer = (float*)dataAt(&uv, index);
				triangleUV[i2][0] = uvPointer[0];
				triangleUV[i2][1] = uvPointer[1];
			} else {
				triangleUV[i2][0] = 0.0F;
				triangleUV[i2][1] = 0.0F;
			}
		}
		fillTriangle(triangle, image, triangleUV, zBuffer, output);
	}
}
