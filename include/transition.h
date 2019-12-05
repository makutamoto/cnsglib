#ifndef TRANSITION_H
#define TRANSITION_H

#include "./graphics.h"

Image linearTransitionH(Image imageA, Image imageB, float ratio);
Image linearTransitionV(Image imageA, Image imageB, float ratio);
Image ThreeDimensionTransition(Image imageA, Image imageB, float x, float y, float ratio);

#endif
