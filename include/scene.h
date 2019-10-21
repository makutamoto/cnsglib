#ifndef SCENE_H
#define SCENE_H

#include<Windows.h>
#include<time.h>

#include "../include/node.h"
#include "../include/vector.h"

typedef struct {
  int positionMask[3];
  float position[3];
  float target[3];
  float worldUp[3];
  float fov;
  float nearLimit;
  float farLimit;
  float aspect;
  Node *parent;
} Camera;

typedef struct {
  float acceleration[3];
  Vector nodes;
  unsigned char background;
  Camera camera;
  Vector intervalEvents;
} Scene;

typedef struct {
	clock_t begin;
	unsigned int interval;
	void (*callback)(Scene*);
} IntervalEventScene;

Camera initCamera(float x, float y, float z, float aspect);

Scene initScene(void);
void addIntervalEventScene(Scene *scene, unsigned int milliseconds, void (*callback)(Scene*));
void drawScene(Scene *scene);
void updateScene(Scene *scene, float elapsed);
void discardScene(Scene *scene);

#endif
