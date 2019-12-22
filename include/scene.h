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
  int isRotationDisabled;
} Camera;

typedef struct _Scene {
  float acceleration[3];
  Vector nodes;
  unsigned char background;
  Camera camera;
  Vector intervalEvents;
  float clock;
  void (*behaviour)(struct _Scene*, float);
} Scene;

typedef struct {
  Node *target;
  Vector info;
} CollisionInfo;

typedef struct {
	clock_t begin;
	unsigned int interval;
	void (*callback)(Scene*);
} IntervalEventScene;

Camera initCamera(float x, float y, float z, float aspect);

Scene initScene(void);
void addIntervalEventScene(Scene *scene, unsigned int milliseconds, void (*callback)(Scene*));
void drawSceneEx(Scene *scene, Image *output, Camera *camera, Node *replacedNode);
void drawScene(Scene *scene, Image *output);
void updateScene(Scene *scene, float elapsed);
void discardScene(Scene *scene);

#endif
