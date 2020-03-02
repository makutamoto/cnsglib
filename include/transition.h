/**
* @file transition.h
* \~english @brief Transition.
* \~japanese @brief トランジション。
*/

#ifndef TRANSITION_H
#define TRANSITION_H

#include "./graphics.h"

Image linearTransitionH(Image *imageA, Image *imageB, float ratio);
Image linearTransitionV(Image *imageA, Image *imageB, float ratio);
void revoluteTransition(Image *image, Image *previous, Image *current, float ratio);
void stripeHTransitionBase(Image *out, Image *previous, Image *current, int n, float ratio, int direction);
void stripeVTransitionBase(Image *out, Image *previous, Image *current, int n, float ratio, int direction);

#endif
