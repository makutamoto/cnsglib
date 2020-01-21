#include "./include/scene.h"
#include "./include/colors.h"
#include "./include/image.h"
#include "./include/controller.h"

static Camera *currentCamera;
static Scene *currentScene;
static Scene *previousScene;
static void (*transitionFunc)(Image*, Image*, Image*, float);
static float transitionSpeed;
static float transitionRatio;
static Image previousImage;

void setCurrentSceneEx(Scene *scene, Camera *camera, void (*transition)(Image*, Image*, Image*, float), float speed) {
  previousScene = currentScene;
  currentScene = scene;
  currentCamera = camera ? camera : &scene->camera;
  transitionFunc = transition;
  transitionSpeed = speed;
  transitionRatio = 0.0F;
}

Scene* getCurrentScene(void) {
  return currentScene;
}

Camera* getCurrentCamera(void) {
  return currentCamera;
}

int drawCurrentScene(Image *image, float elapsed) {
  if(currentScene) {
    drawSceneEx(currentScene, image, currentCamera, NULL);
    if(previousScene && transitionFunc && transitionRatio < 1.0F) {
      Image currentImage;
      copyImage(&currentImage, image);
      transitionFunc(image, &previousImage, &currentImage, transitionRatio);
      transitionRatio += transitionSpeed * elapsed;
      freeImage(&currentImage);
      return FALSE;
    } else {
      if(previousImage.width != 0) freeImage(&previousImage);
      copyImage(&previousImage, image);
    }
    return TRUE;
  }
  return FALSE;
}

void updateCurrentController(int update) {
  if(currentScene) {
    updateController(&currentCamera->controllerList, update);
  }
}

void updateCurrentScene(float elapsed) {
  if(currentScene) {
    updateSceneEx(currentScene, elapsed / 2.0F, currentCamera);
    updateSceneEx(currentScene, elapsed / 2.0F, currentCamera);
  }
}
