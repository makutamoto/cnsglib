/**
* @file sound.h
* \~english @brief Sound.
* \~japanese @brief âπÅB
*/

#ifndef SOUND_H
#define SOUND_H

#include<Windows.h>

#define Sound HANDLE

void initSound(int argc, char *argv[]);
Sound PlaySoundNeo(const char path[], int loop);
void StopSound(Sound sound);
void deinitSound(void);

#endif
