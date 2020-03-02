#ifndef MANAGER_H
#define MANAGER_H

#include "./vector.h"
#include "./image.h"
#include "./scene.h"

typedef struct {
  float x;
  float y;
  float width;
  float height;
  unsigned char transparent;
  int isUpdateDisabled;
  int isSceneUpdateDisabled;
  Camera *currentCamera;
  Scene *currentScene;
  Scene *previousScene;
  void (*transitionFunc)(Image*, Image*, Image*, float);
  float transitionSpeed;
  float transitionRatio;
  Image previousImage;
} View;

typedef struct {
  Vector views;
  Vector controllerList;
} Window;

typedef struct {
  Window *currentWindow;
  Window *previousWindow;
  void (*transitionFunc)(Image*, Image*, Image*, float);
  float transitionSpeed;
  float transitionRatio;
  Image previousImage;
} WindowManager;

#define setViewScene(view, scene, transition, speed) setViewSceneEx(view, scene, NULL, transition, speed)
#define isViewInTransition(view) (view->previousScene && view->transitionFunc && view->transitionRatio < 1.0F)
#define isWindowManagerInTransition(manager) (manager->previousWindow && manager->transitionFunc && manager->transitionRatio < 1.0F)

void setDebugMode(int mode);
int getDebugMode(void);
void setViewSceneEx(View *view, Scene *scene, Camera *camera, void (*transition)(Image*, Image*, Image*, float), float speed);
void drawView(View *view, Image *image, float elapsed);
void updateViewController(View *view);
void updateView(View *view, float elapsed);
Window initWindow(void);
View initView(Scene *scene);
void drawWindow(Window *window, Image *image, float elapsed);
void updateWindowController(Window *window);
void updateWindow(Window *window, float elapsed);

WindowManager initWindowManager(Window *window);
void drawWindowManager(WindowManager *manager, Image *image, float elapsed);
void setWindow(WindowManager *manager, Window *window, void (*transition)(Image*, Image*, Image*, float), float speed);
void updateWindowManagerController(WindowManager *manager);
void updateWindowManager(WindowManager *manager, float elapsed);

#endif
