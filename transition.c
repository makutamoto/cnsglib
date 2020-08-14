#include<stdio.h>
#include<math.h>

#include "./include/borland.h"
#include "./include/graphics.h"
#include "./include/transition.h"
#include "./include/scene.h"
#include "./include/node.h"
#include "./include/matrix.h"
#include "./include/colors.h"

Image linearTransitionH(Image *imageA, Image *imageB, float ratio) {
  Image output = initImageBulk(imageA->width, imageA->height, imageA->transparent);
  if(isImageSameSize(*imageA, *imageB)) {
    unsigned int row, col;
    unsigned int aWidth = output.width * min(max(1.0F - ratio, 0.0F), 1.0F);
    for(row = 0;row < output.height;row++) {
      for(col = 0;col < output.width;col++) {
        size_t index = row * output.width + col;
        output.data[index] = (col < aWidth) ? imageA->data[index] : imageB->data[index];
      }
    }
  } else {
    fprintf(stderr, "linearTransitionH(): The image size must be the same.\n");
  }
  return output;
}

Image linearTransitionV(Image *imageA, Image *imageB, float ratio) {
  Image output = initImageBulk(imageA->width, imageA->height, imageA->transparent);
  if(isImageSameSize(*imageA, *imageB)) {
    unsigned int row, col;
    unsigned int aWidth = output.height * min(max(1.0F - ratio, 0.0F), 1.0F);
    for(col = 0;col < output.width;col++) {
      for(row = 0;row < output.height;row++) {
        size_t index = row * output.width + col;
        output.data[index] = (row < aWidth) ? imageA->data[index] : imageB->data[index];
      }
    }
  } else {
    fprintf(stderr, "linearTransitionV(): The image size must be the same.\n");
  }
  return output;
}

void revoluteTransition(Image *out, Image *previous, Image *current, float ratio) {
  unsigned int row, col;
  float angle;
  int halfWidth, halfHeight;
  int inverse;
  Image *image;
  if(ratio < 0.5F) {
    inverse = FALSE;
    ratio = ratio / 0.5F;
    image = previous;
  } else {
    inverse = TRUE;
    ratio = (ratio - 0.5F) / 0.5F;
    image = current;
  }
  ratio = max(min(ratio, 1.0F), 0.0F);
  angle = PI / 2.0F * ratio;
  halfWidth = image->width / 2;
  halfHeight = image->height / 2;
  for(row = 0;row < out->height;row++) {
    for(col = 0;col < out->width;col++) {
      long index = row * image->width + col;
      int x, y;
      x = col - halfWidth;
      y = row - halfHeight;
      if(x == 0 || y == 0) {
        out->data[index] = BLACK;
      } else {
        if(sign(x) == sign(y)) {
          if(angle <= fabsf(atanf((float)y / x))) {
            out->data[index] = inverse ? BLACK : image->data[index];
          } else {
            out->data[index] = inverse ? image->data[index] : BLACK;
          }
        } else {
          if(angle <= fabsf(PI / 2.0F + atanf((float)y / x))) {
            out->data[index] = inverse ? BLACK : image->data[index];
          } else {
            out->data[index] = inverse ? image->data[index] : BLACK;
          }
        }
      }
    }
  }
}

void stripeHTransitionBase(Image *out, Image *previous, Image *current, int n, float ratio, int direction) {
  int row, col;
  int stride;
  int inverse;
  Image *image;
  if(ratio < 0.5F) {
    inverse = TRUE;
    ratio = ratio / 0.5F;
    image = previous;
  } else {
    inverse = FALSE;
    ratio = (ratio - 0.5F) / 0.5F;
    image = current;
  }
  stride = ceilf((float)image->width / n);
  ratio = max(min(ratio, 1.0F), 0.0F);
  for(row = 0;row < (int)out->height;row++) {
    for(col = 0;col < (int)out->width;col++) {
      long index = row * image->width + col;
      if((direction ? stride - col % stride : col % stride) <= ratio * stride) {
        out->data[index] = inverse ? BLACK : image->data[index];
      } else {
        out->data[index] = inverse ? image->data[index] : BLACK;
      }
    }
  }
}

void stripeVTransitionBase(Image *out, Image *previous, Image *current, int n, float ratio, int direction) {
  int row, col;
  int stride;
  int inverse;
  Image *image;
  if(ratio < 0.5F) {
    inverse = TRUE;
    ratio = ratio / 0.5F;
    image = previous;
  } else {
    inverse = FALSE;
    ratio = (ratio - 0.5F) / 0.5F;
    image = current;
  }
  stride = ceilf((float)image->height / n);
  ratio = max(min(ratio, 1.0F), 0.0F);
  for(row = 0;row < (int)out->height;row++) {
    for(col = 0;col < (int)out->width;col++) {
      long index = row * image->width + col;
      if((direction ? stride - row % stride : row % stride) <= ratio * stride) {
        out->data[index] = inverse ? BLACK : image->data[index];
      } else {
        out->data[index] = inverse ? image->data[index] : BLACK;
      }
    }
  }
}
