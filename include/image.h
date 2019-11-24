#ifndef IMAGE_H
#define IMAGE_H

typedef struct {
	unsigned int width;
	unsigned int height;
	unsigned char transparent;
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

#define isImageSameSize(imageA, imageB) (imageA.width == imageB.width && imageA.height == imageB.height)

unsigned long uvToIndex(const Image *image, float uv[2]);
Image initImage(unsigned int width, unsigned int height, unsigned char color, unsigned char transparent);
Image initImageBulk(unsigned int width, unsigned int height, unsigned char transparent);
void clearImage(Image *image, unsigned char color);
void cropImage(Image dest, Image src, unsigned int xth, unsigned int yth);
void pasteImage(Image dest, Image src, unsigned int x, unsigned int y);
Image loadBitmap(char *fileName, unsigned char transparent);
Image genRect(unsigned int width, unsigned int height, unsigned char color);
Image genCircle(unsigned int rad, unsigned char color);
void freeImage(Image image);

FontSJIS initFontSJIS(Image font0201, Image font0208, unsigned int width0201, unsigned int width0208, unsigned int height);
BOOL drawCharSJIS(Image target, FontSJIS font, unsigned int x, unsigned int y, char *character);
void drawTextSJIS(Image target, FontSJIS font, unsigned int x, unsigned int y, char *text);

#endif
