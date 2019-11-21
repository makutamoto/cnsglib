#ifndef IMAGE_H
#define IMAGE_H

typedef struct {
	unsigned int width;
	unsigned int height;
	unsigned char transparent;
	unsigned char *data;
} Image;

#define isImageSameSize(imageA, imageB) (imageA.width == imageB.width && imageA.height == imageB.height)

unsigned long uvToIndex(const Image *image, float uv[2]);

#endif
