#include<stdio.h>

#include "./include/graphics.h"
#include "./include/transition.h"

Image linearTransition(Image imageA, Image imageB, float ratio) {
  if(ratio == 0.0F) {
    return imageA;
  } else if(ratio == 1.0F) {
    return imageB;
  } else {
    Image output = initImageBulk(imageA.width, imageA.height, imageA.transparent);
    if(imageA.width == imageB.width && imageA.height == imageB.height) {
      unsigned int row, col;
      unsigned int aWidth = output.width * min(max(1.0F - ratio, 0.0F), 1.0F);
      for(row = 0;row < output.height;row++) {
        for(col = 0;col < output.width;col++) {
          size_t index = row * output.width + col;
          output.data[index] = (col < aWidth) ? imageA.data[index] : imageB.data[index];
        }
      }
    } else {
      fprintf(stderr, "linearTransition(): The image size must be the same.\n");
    }
    return output;
  }
}
