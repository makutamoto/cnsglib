/**
* @file bitmap.h
* \~english @brief Defines the structure of bimap images.
* \~japanese @brief ビットマップ画像の構造を定義します。
*/

#ifndef BITMAP_H
#define BITMAP_H

#include "./borland.h"

#ifndef __BORLANDC__
#include<stdint.h>
#endif

#pragma pack(1)

/**
* \~ @brief Bitmap Header.
*/
typedef struct {
  volatile char magicNumber[2];
  volatile uint32_t size;
  volatile uint32_t reserved;
  volatile uint32_t offset;
  volatile uint32_t dibSize;
} BitmapHeader;

/**
* \~ @brief Bitmap Info Header.
*/
typedef struct {
  int32_t width;
  int32_t height;
  uint16_t nofColorPlanes;
  uint16_t nofBitsPPixel;
  uint32_t compressionMethod;
  uint32_t imageSize;
  uint32_t hResolution;
  uint32_t vResolution;
  uint32_t nofColors;
  uint32_t nofImportantColors;
} BitmapInfoHeader;

typedef struct {
  char magicNumber[2];
  uint32_t size;
  uint32_t reserved;
  uint32_t offset;
  uint32_t dibSize;
  int32_t width;
  int32_t height;
  uint16_t nofColorPlanes;
  uint16_t nofBitsPPixel;
  uint32_t compressionMethod;
  uint32_t imageSize;
  uint32_t hResolution;
  uint32_t vResolution;
  uint32_t nofColors;
  uint32_t nofImportantColors;
  uint32_t palettes[16];
} BitmapHeaderOut;

#pragma pack()

#endif
