#ifndef MANAGER_H
#define MANAGER_H

#define setCurrentScene(scene, transition, speed) setCurrentSceneEx(scene, NULL, transition, speed)

void setCurrentSceneEx(Scene *scene, Camera *camera, void (*transition)(Image*, Image*, Image*, float), float speed);
int drawCurrentScene(Image *image, float elapsed);
void updateCurrentScene(float elapsed);

#endif
