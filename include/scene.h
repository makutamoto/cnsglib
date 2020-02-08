/**
* @file scene.h
* \~english @brief Scene.
* \~japanese @brief ÉVÅ[ÉìÅB
*/

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
  Vector controllerList;
  unsigned char sceneFilterAND;
  unsigned char sceneFilterOR;
  int isRotationDisabled;
} Camera;

typedef struct Scene {
  float acceleration[3];
  Vector nodes;
  unsigned char background;
  Camera camera;
  Vector intervalEvents;
  float clock;
  float speed;
  void (*behaviour)(struct Scene*, float);
} Scene;

typedef struct {
  Node *target;
  Vector info;
} CollisionInfo;

typedef struct {
  float counter;
	float seconds;
  void *data;
	int (*callback)(Scene*, void*);
} IntervalEventScene;

#define drawScene(scene, output) drawSceneEx((scene), (output), &(scene)->camera, NULL)
#define updateScene(scene, elapsed) updateSceneEx(scene, elapsed, &(scene)->camera)

Camera initCamera(float x, float y, float z);

Scene initScene(void);
IntervalEventScene* addIntervalEventScene(Scene *scene, float seconds, int (*callback)(Scene*, void*), void *data);
Node* getNodeByMask(Vector *nodes, unsigned int mask);
void drawSceneEx(Scene *scene, Image *output, Camera *camera, Node *replacedNode);
void updateSceneEx(Scene *scene, float rawElapsed, Camera *camera);
void discardScene(Scene *scene);

#endif
