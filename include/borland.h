/**
* @file borland.h
* \~english @brief Defines functions and macros for bcc55 compatibility.
* \~japanese @brief BCC55å›ä∑ÇÃÇΩÇﬂÇÃíËã`ÅB
*/

#ifdef __BORLANDC__
#ifndef BORLAND_H
#define BORLAND_H

#include<stdio.h>
#include<time.h>

typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef int int32_t;
typedef unsigned int uint32_t;

#define atanf(x) (float)atan(x)
#define ceilf(x) (float)ceil(x)
#define fabsf(x) (float)fabs(x)
#define floorf(x) (float)floor(x)
#define cosf(x) (float)cos(x)
#define sinf(x) (float)sin(x)
#define sqrtf(x) (float)sqrt(x)
#define tanf(x) (float)tan(x)
float roundf(float x);

int fopen_s(FILE** pFile, const char *filename, const char *mode);
#define fread_s(buffer, bufferSize, elementSize, count, stream) fread(buffer, elementSize, count, stream)

int localtime_s(struct tm* const tmDest, time_t const* const sourceTime);

#define memcpy_s(dest, destSize, src, count) memcpy(dest, src, count)
#define strcat_s(dest, size, src) strcat(dest, src)
#define strcpy_s(dest, size, src) strcpy(dest, src)

#endif
#endif
