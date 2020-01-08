#include<Windows.h>
#include<stdio.h>
#include<math.h>

#include "./include/image.h"
#include "./include/bitmap.h"
#include "./include/colors.h"

Image NO_IMAGE;
Image TRANSPARENT_IMAGE;
Image BLACK_IMAGE;

void initColorImages(void) {
  TRANSPARENT_IMAGE = initImage(1, 1, BLACK, BLACK);
  BLACK_IMAGE = initImage(1, 1, BLACK, NULL_COLOR);
}

unsigned long uvToIndex(const Image *image, float uv[2]) {
  unsigned int x, y;
  x = (image->width - 1) * uv[0];
  y = (image->height - 1) * uv[1];
  return image->width * y + x;
}

Image initImage(unsigned int width, unsigned int height, unsigned char color, unsigned char transparent) {
	Image image;
	size_t imageSize = width * height;
	image.width = width;
	image.height = height;
	image.transparent = transparent;
  image.transparentFilter = 0x0F;
	image.data = malloc(imageSize);
	memset(image.data, color, imageSize);
	return image;
}

Image initImageBulk(unsigned int width, unsigned int height, unsigned char transparent) {
	Image image;
	size_t imageSize = width * height;
	image.width = width;
	image.height = height;
	image.transparent = transparent;
  image.transparentFilter = 0x0F;
	image.data = malloc(imageSize);
	return image;
}

void clearImage(Image *image, unsigned char color) {
	memset(image->data, color, image->width * image->height);
}

int isImageOverlap(Image *imageA, Image *imageB) {
  unsigned int x, y;
  if(!isImageSameSize(*imageA, *imageB)) {
    fprintf(stderr, "isImageOverlap: the image sizes must be the same.\n");
    return FALSE;
  }
  for(x = 0;x < imageA->width;x++) {
    for(y = 0;y < imageA->height;y++) {
      size_t index = x + y * imageA->width;
      if(imageA->data[index] == BLACK && imageB->data[index] == BLACK) return TRUE;
    }
  }
  return FALSE;
}

void replaceImageColor(Image *image, unsigned char from, unsigned char to) {
  unsigned int x, y;
  for(x = 0;x < image->width;x++) {
    for(y = 0;y < image->height;y++) {
      size_t index = x + y * image->width;
      if(image->data[index] == from) image->data[index] = to;
    }
  }
}

void cropImage(Image *dest, Image *src, unsigned int xth, unsigned int yth) {
	size_t ix, iy;
	unsigned int x = dest->width * xth;
	unsigned int y = dest->height * yth;
  dest->transparent = src->transparent;
  dest->transparentFilter = src->transparentFilter;
	if(dest->width > src->width || dest->height > src->height) {
		fprintf(stderr, "cropImage: the cropped size is bigger than original size.\n");
	}
	if(xth >= src->width / dest->width || yth >= src->height / dest->height) {
		fprintf(stderr, "cropImage: the position is out of range: (%u, %u)\n", xth, yth);
	}
	for(iy = 0;iy + y < src->height && iy < dest->height;iy++) {
		for(ix = 0;ix + x < src->width && ix < dest->width;ix++) {
			dest->data[dest->width * iy + ix] = src->data[src->width * (iy + y) + ix + x];
		}
	}
}

void pasteImage(Image dest, Image src, int x, int y) {
	long ix, iy;
	for(iy = 0;iy < (long)src.height && iy + y < (long)dest.height;iy++) {
		for(ix = 0;ix < (long)src.width && ix + x < (long)dest.width;ix++) {
      size_t srcIndex = src.width * iy + ix;
      if(iy + y >= 0 && ix + x >= 0) {
        size_t destIndex = dest.width * (iy + y) + ix + x;
        if(src.data[srcIndex] == src.transparent) {
          dest.data[destIndex] &= src.transparentFilter;
        } else {
          dest.data[destIndex] = src.data[srcIndex];
        }
      }
		}
	}
}

void copyImage(Image *dest, Image *src) {
  size_t size;
  size = src->width * src->height;
  dest->width = src->width;
  dest->height = src->height;
  dest->transparent = src->transparent;
  dest->transparentFilter = src->transparentFilter;
  dest->data = malloc(size);
  memcpy_s(dest->data, size, src->data, size);
}

FontSJIS initFontSJIS(Image font0201, Image font0208, unsigned int width0201, unsigned int width0208, unsigned int height) {
	FontSJIS font;
	font.font0201 = font0201;
	font.font0208 = font0208;
	font.offset0201 = 0;
	font.offset0208 = 0;
	font.height = height;
	font.width[0] = width0201;
	font.width[1] = width0208;
	return font;
}

BOOL drawCharSJIS(Image target, FontSJIS font, unsigned int x, unsigned int y, char *character) {
	unsigned int fontx, fonty;
	int ismultibyte;
	Image image;
	if(isCharacterMultibyte((unsigned char)character[0])) {
		unsigned int multibyte = (unsigned char)character[0] << 8 | (unsigned char)character[1];
		fontx = multibyte & 0x0F;
		fonty = ((multibyte - 0x8140) >> 4) - font.offset0208;
		ismultibyte = TRUE;
		image = initImage(font.width[1], font.height, BLACK, NULL_COLOR);
		cropImage(&image, &font.font0208, fontx, fonty);
	} else {
		fontx = character[0] & 0x0F;
		fonty = ((character[0] >> 4) - font.offset0201) & 0x0F;
    ismultibyte = FALSE;
		image = initImage(font.width[0], font.height, BLACK, NULL_COLOR);
		cropImage(&image, &font.font0201, fontx, fonty);
	}
	pasteImage(target, image, x, y);
	freeImage(image);

	return ismultibyte;
}

void drawTextSJIS(Image target, FontSJIS font, unsigned int x, unsigned int y, char *text) {
	unsigned int dx = 0;
	unsigned int dy = 0;
	while(*text != '\0') {
		switch(*text) {
			case '\n':
				dx = 0;
				dy += font.height;
				break;
			case '\r':
				break;
			default:
				if(drawCharSJIS(target, font, x + dx, y + dy, text)) {
					dx += font.width[1];
					text += 1;
				} else {
					dx += font.width[0];
				}
		}
		text += 1;
	}
}

Image loadBitmap(char *fileName, unsigned char transparent) {
	Image image = { 0, 0, NULL_COLOR, 0, NULL };
	FILE *file;
	BitmapHeader header;
	BitmapInfoHeader infoHeader;
	uint32_t *img;
	unsigned int y;
	if(fopen_s(&file, fileName, "rb")) {
		fprintf(stderr, "Failed to open the file '%s'\n", fileName);
		fclose(file);
		return image;
	}
	if(fread_s(&header, sizeof(BitmapHeader), 1, sizeof(BitmapHeader), file) != sizeof(BitmapHeader)) {
		fprintf(stderr, "BMP header does not exist in the file '%s'\n", fileName);
		fclose(file);
		return image;
	}
	if(header.magicNumber[0] != 'B' || header.magicNumber[1] != 'M') {
		fprintf(stderr, "Unknown magic number.\n");
		fclose(file);
		return image;
	}
	if(header.dibSize != 40) {
		fprintf(stderr, "Unknown DIB header type.\n");
		fclose(file);
		return  image;
	}
	if(fread_s(&infoHeader, sizeof(BitmapInfoHeader), 1, sizeof(BitmapInfoHeader), file) != sizeof(BitmapInfoHeader)) {
		fprintf(stderr, "Failed to load Bitmap info header.\n");
		fclose(file);
		return image;
	}
	if(infoHeader.width < 0 || infoHeader.height < 0) {
		fprintf(stderr, "Negative demensions are not supported.\n");
		fclose(file);
		return image;
	}
	if(infoHeader.compressionMethod != 0) {
		fprintf(stderr, "Compressions are not supported.\n");
		fclose(file);
		return image;
	}
	if(fseek(file, (long)header.offset, SEEK_SET)) {
		fprintf(stderr, "Image data does not exist.\n");
		fclose(file);
		return image;
	}
	img = (uint32_t*)malloc(infoHeader.imageSize);
	if(fread_s(img, infoHeader.imageSize, 1, infoHeader.imageSize, file) != infoHeader.imageSize) {
		fprintf(stderr, "Image data is corrupted.\n");
		fclose(file);
		return image;
	}
	image.width = (unsigned int)infoHeader.width;
	image.height = (unsigned int)infoHeader.height;
	image.transparent = transparent;
  image.transparentFilter = 0x0F;
	image.data = (unsigned char*)malloc(image.width * image.height);
	for(y = 0;y < image.height;y++) {
		unsigned int x;
		for(x = 0;x < image.width;x++) {
			size_t index = y * (unsigned int)(ceilf(image.width / 8.0F)) + x / 8;
			uint8_t location = x % 8;
			uint8_t highLow = location % 2;
			uint8_t color;
			if(highLow) {
				color = (img[index] >> ((location - 1) * 4)) & 0x0F;
			} else {
				color = (img[index] >> ((location + 1) * 4)) & 0x0F;
			}
			image.data[image.width * (image.height - 1 - y) + x] = color;
		}
	}
	fclose(file);
	return image;
}

void drawRect(Image *image, int x, int y, int width, int height, unsigned char color) {
	Image rect;
  rect = initImage(width, height, color, NULL_COLOR);
  pasteImage(*image, rect, x, y);
  freeImage(rect);
}

Image genCircle(unsigned int radius, unsigned char color) {
	Image image;
	unsigned int edgeWidth = 2 * radius;
	unsigned int y;
	image.data = malloc(edgeWidth * edgeWidth);
	if(image.data == NULL) {
		image.width = 0;
		image.height = 0;
		image.transparent = NULL_COLOR;
    image.transparentFilter = 0x0F;
		image.data = NULL;
		return image;
	}
	image.width = edgeWidth;
	image.height = edgeWidth;
	image.transparent = color ^ 0x0F;
  image.transparentFilter = 0x0F;
	for(y = 0;y < edgeWidth;y++) {
		unsigned int x;
		for(x = 0;x < edgeWidth;x++) {
			int cx = (int)x - (int)radius;
			int cy = (int)y - (int)radius;
			size_t index = edgeWidth * y + x;
			if(radius * radius >= (unsigned int)(cx * cx + cy * cy)) {
				image.data[index] = color;
			} else {
				image.data[index] = image.transparent;
			}
		}
	}
	return image;
}

void freeImage(Image image) {
	free(image.data);
}
