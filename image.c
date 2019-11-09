#include "./include/image.h"

unsigned long uvToIndex(const Image *image, float uv[2]) {
  unsigned int x, y;
  x = (image->width - 1) * uv[0];
  y = (image->height - 1) * uv[1];
  return image->width * y + x;
}
