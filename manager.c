#include "./include/manager.h"
#include "./include/scene.h"
#include "./include/colors.h"
#include "./include/image.h"
#include "./include/controller.h"

static int debugMode;

void setDebugMode(int mode) {
  debugMode = mode;
}

int getDebugMode(void) {
  return debugMode;
}

void setViewSceneEx(View *view, Scene *scene, Camera *camera, void (*transition)(Image*, Image*, Image*, float), float speed) {
  view->previousScene = view->currentScene;
  view->currentScene = scene;
  view->currentCamera = camera ? camera : &scene->camera;
  view->transitionFunc = transition;
  view->transitionSpeed = speed;
  view->transitionRatio = 0.0F;
}

void drawView(View *view, Image *image, float elapsed) {
  if(view->currentScene) {
    drawSceneEx(view->currentScene, image, view->currentCamera, NULL);
    if(isViewInTransition(view)) {
      Image currentImage;
      copyImage(&currentImage, image);
      view->transitionFunc(image, &view->previousImage, &currentImage, view->transitionRatio);
      view->transitionRatio += view->transitionSpeed * elapsed;
      freeImage(&currentImage);
    } else {
      if(view->previousImage.width != 0) freeImage(&view->previousImage);
      copyImage(&view->previousImage, image);
    }
  }
}

void updateViewController(View *view) {
  if(view->currentScene) registerControllerList(&view->currentCamera->controllerList, !isViewInTransition(view));
}

void updateView(View *view, float elapsed) {
  if(view->currentScene) {
    if(view->isSceneUpdateDisabled) {
      float time = elapsed / 2.0F;
      updateCamera(view->currentCamera, time);
      updateCamera(view->currentCamera, time);
    } else if(!view->isUpdateDisabled) {
      float time = elapsed / 2.0F;
      updateSceneEx(view->currentScene, time, view->currentCamera);
      updateSceneEx(view->currentScene, time, view->currentCamera);
    }
  }
}

Window initWindow(void) {
  Window window;
  memset(&window, 0, sizeof(Window));
  window.views = initVector();
  window.controllerList = initVector();
  return window;
}

View initView(Scene *scene) {
  View view;
  memset(&view, 0, sizeof(View));
  view.width = 1.0F;
  view.height = 1.0F;
  view.transparent = NULL_COLOR;
  setViewScene(&view, scene, NULL, 0.0F);
  return view;
}

void drawWindow(Window *window, Image *image, float elapsed) {
  View *view;
  iterf(&window->views, &view) {
    Image region;
    region = initImageBulk(image->width * view->width, image->height * view->height, view->transparent);
    drawView(view, &region, elapsed);
    pasteImage(image, &region, image->width * view->x, image->height * view->y);
    freeImage(&region);
  }
}

void updateWindowController(Window *window) {
  View *view;
  iterf(&window->views, &view) updateViewController(view);
  registerControllerList(&window->controllerList, TRUE);
  updateController();
}

void updateWindow(Window *window, float elapsed) {
  View *view;
  iterf(&window->views, &view) updateView(view, elapsed);
}

WindowManager initWindowManager(Window *window) {
  WindowManager manager;
  memset(&manager, 0, sizeof(WindowManager));
  setWindow(&manager, window, NULL, 0.0F);
  return manager;
}

void drawWindowManager(WindowManager *manager, Image *image, float elapsed) {
  if(manager->currentWindow) {
    drawWindow(manager->currentWindow, image, elapsed);
    if(isWindowManagerInTransition(manager)) {
      Image currentImage;
      copyImage(&currentImage, image);
      manager->transitionFunc(image, &manager->previousImage, &currentImage, manager->transitionRatio);
      manager->transitionRatio += manager->transitionSpeed * elapsed;
      freeImage(&currentImage);
    } else {
      if(manager->previousImage.width != 0) freeImage(&manager->previousImage);
      copyImage(&manager->previousImage, image);
    }
  }
}

void setWindow(WindowManager *manager, Window *window, void (*transition)(Image*, Image*, Image*, float), float speed) {
  manager->previousWindow = manager->currentWindow;
  manager->currentWindow = window;
  manager->transitionFunc = transition;
  manager->transitionSpeed = speed;
  manager->transitionRatio = 0.0F;
}

void updateWindowManagerController(WindowManager *manager) {
  if(manager->currentWindow) updateWindowController(manager->currentWindow);
}

void updateWindowManager(WindowManager *manager, float elapsed) {
  if(manager->currentWindow) updateWindow(manager->currentWindow, elapsed);
}
