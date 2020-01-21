#ifndef MANAGER_H
#define MANAGER_H

#define setCurrentScene(scene, transition, speed) setCurrentSceneEx(scene, NULL, transition, speed)

void setCurrentSceneEx(Scene *scene, Camera *camera, void (*transition)(Image*, Image*, Image*, float), float speed);
Scene* getCurrentScene(void);
Camera* getCurrentCamera(void);
int drawCurrentScene(Image *image, float elapsed);
void updateCurrentController(int update);
void updateCurrentScene(float elapsed);

#endif
