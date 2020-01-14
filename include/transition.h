/**
* @file transition.h
* \~english @brief Transition.
* \~japanese @brief トランジション。
*/

#ifndef TRANSITION_H
#define TRANSITION_H

#include "./graphics.h"

Image linearTransitionH(Image imageA, Image imageB, float ratio);
Image linearTransitionV(Image imageA, Image imageB, float ratio);
void revoluteTransition(Image *image, float ratio, int inverse, Image *out);
void stripeTransitionH(Image *image, int n, float ratio, int direction, int inverse, Image *out);
void stripeTransitionV(Image *image, int n, float ratio, int direction, int inverse, Image *out);
Image ThreeDimensionTransition(Image imageA, Image imageB, float x, float y, float ratio);

#endif
