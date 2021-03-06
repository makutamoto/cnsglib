/**
* @file image.h
* \~english @brief Defines functions to deal with images.
* \~japanese @brief 画像を扱うための関数定義。
*/

#ifndef IMAGE_H
#define IMAGE_H

#include<Windows.h>

typedef struct {
	unsigned int width;
	unsigned int height;
	unsigned char transparent;
	unsigned char transparentFilter;
	unsigned char *data;
} Image;

typedef struct {
	Image font0201;
	Image font0208;
	unsigned int offset0201;
	unsigned int offset0208;
	unsigned int height;
	unsigned int width[2];
} FontSJIS;

extern Image NO_IMAGE;
extern Image TRANSPARENT_IMAGE;
extern Image BLACK_IMAGE;

#define isImageSameSize(imageA, imageB) ((imageA).width == (imageB).width && (imageA).height == (imageB).height)
#define isCharacterMultibyte(character) (((character) >= 0x81 && (character) <= 0x9F) || ((character) >= 0xE0 && (character) <= 0xEF))

#define loadBitmap(file, transparent) loadBitmapEx(file, transparent, FALSE)
#define saveBitmap(image, path) saveBitmapEx((image), (path), FALSE)

void initColorImages(void);
unsigned long uvToIndex(const Image *image, float uv[2]);
Image initImage(unsigned int width, unsigned int height, unsigned char color, unsigned char transparent);
Image initImageBulk(unsigned int width, unsigned int height, unsigned char transparent);
void clearImage(Image *image, unsigned char color);
int isImageOverlap(Image *imageA, Image *imageB);
void replaceImageColor(Image *image, unsigned char from, unsigned char to);
void cropImage(Image *dest, Image *src, unsigned int xth, unsigned int yth);
void pasteImage(Image *dest, Image *src, int x, int y);
void copyImage(Image *dest, Image *src);
Image loadBitmapEx(char *fileName, unsigned char transparent, int allowNotFound);
int saveBitmapEx(Image *image, char *path, int allowError);
void drawRect(Image *image, int x, int y, int width, int height, unsigned char color);
void drawCircle(Image *image, int px, int py, int radius, unsigned char color);
void freeImage(Image *image);

FontSJIS initFontSJIS(Image font0201, Image font0208, unsigned int width0201, unsigned int width0208, unsigned int height);
BOOL drawCharSJIS(Image *target, FontSJIS *font, unsigned int x, unsigned int y, char *character);
void drawTextSJIS(Image *target, FontSJIS *font, unsigned int x, unsigned int y, char *text);

#endif
