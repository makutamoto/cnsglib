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
  Vector nodes;
  unsigned char sceneFilterAND;
  unsigned char sceneFilterOR;
  int isRotationDisabled;
} Camera;

typedef struct {
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
	int (*callback)(Scene*);
} IntervalEventScene;

#define drawScene(scene, output) drawSceneEx((scene), (output), &(scene)->camera, NULL)
#define updateScene(scene, elapsed) updateSceneEx(scene, elapsed, &(scene)->camera)

void initCamera(Camera *camera, float x, float y, float z);

Scene initScene(void);
void addIntervalEventScene(Scene *scene, unsigned int milliseconds, int (*callback)(Scene*));
void drawSceneEx(Scene *scene, Image *output, Camera *camera, Node *replacedNode);
void updateSceneEx(Scene *scene, float elapsed, Camera *camera);
void discardScene(Scene *scene);

#endif
