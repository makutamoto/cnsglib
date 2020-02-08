#include<Windows.h>
#include<time.h>
#include<math.h>

#include "./include/borland.h"
#include "./include/scene.h"
#include "./include/node.h"
#include "./include/vector.h"
#include "./include/matrix.h"
#include "./include/graphics.h"
#include "./include/colors.h"
#include "./include/manager.h"

#define VELOCITY_LIMIT 200.0F * 10000.0F / 3600.0F

void initCamera(Camera *camera, float x, float y, float z) {
  memset(camera, 0, sizeof(Camera));
  camera->position[0] = x;
  camera->position[1] = y;
  camera->position[2] = z;
  camera->worldUp[1] = 1.0F;
  camera->fov = PI / 3.0F * 2.0F;
  camera->nearLimit = 10.0F;
  camera->farLimit = 1000.0F;
  camera->sceneFilterAND = 0x0F;
}

Scene initScene(void) {
  Scene scene;
  memset(&scene, 0, sizeof(Scene));
  scene.acceleration[1] = -98.0F;
  scene.nodes = initVector();
  scene.speed = 1.0F;
  initCamera(&scene.camera, 0.0F, 0.0F, 0.0F);
  return scene;
}

IntervalEventScene* addIntervalEventScene(Scene *scene, float seconds, int (*callback)(Scene*, void*), void *data) {
  IntervalEventScene *interval = malloc(sizeof(IntervalEventScene));
  interval->counter = 0.0F;
  interval->seconds = seconds;
  interval->data = data;
  interval->callback = callback;
  push(&scene->intervalEvents, interval);
  return interval;
}

Node* getNodeByMask(Vector *nodes, unsigned int mask) {
  VectorIter iter;
  CollisionInfo *info;
  iter = initVectorIter(nodes);
  for(info = nextDataIter(&iter);info;info = nextDataIter(&iter)) {
    if(info->target->collisionMaskActive & mask) return info->target;
  }
  return NULL;
}

void drawSceneEx(Scene *scene, Image *output, Camera *camera, Node *replacedNode) {
  Node *node;
  float lookAt[4][4];
  float projection[4][4];
  float cameraMatrix[4][4];
  float *zBuffer;
  float aspect;
  int debugMode;
  debugMode = getDebugMode();
  if(output->width == 0 || output->height == 0) {
    aspect = 1.0F;
  } else {
    aspect = camera->aspect == 0.0F ? (float)output->width / output->height : camera->aspect;
  }
  clearTransformation();
  if(camera->parent) {
    float temp[4];
    float tempMat4[1][4][4];
    float cameraParent[4][4];
    float cameraPosition[4];
    float cameraTarget[4];
    getWorldTransfomration(camera->parent, cameraParent);
    if(camera->isRotationDisabled) {
      genIdentityMat4(tempMat4[0]);
      tempMat4[0][0][3] = cameraParent[0][3];
      tempMat4[0][1][3] = cameraParent[1][3];
      tempMat4[0][2][3] = cameraParent[2][3];
      memcpy_s(cameraParent, SIZE_MAT4, tempMat4[0], SIZE_MAT4);
    }
    mulMat4Vec4(cameraParent, convVec3toVec4(camera->position, temp), cameraPosition);
    if(camera->positionMask[0]) cameraPosition[0] = camera->parent->position[0] + camera->position[0];
    if(camera->positionMask[1]) cameraPosition[1] = camera->parent->position[1] + camera->position[1];
    if(camera->positionMask[2]) cameraPosition[2] = camera->parent->position[2] + camera->position[2];
    mulMat4Vec4(cameraParent, convVec3toVec4(camera->target, temp), cameraTarget);
    genLookAtMat4(cameraPosition, cameraTarget, camera->worldUp, lookAt);
  } else {
    genLookAtMat4(camera->position, camera->target, camera->worldUp, lookAt);
  }
  genPerspectiveMat4(camera->fov, camera->nearLimit, camera->farLimit, aspect, projection);
  mulMat4(projection, lookAt, cameraMatrix);
  setZNear(camera->nearLimit);
  clearImage(output, debugMode ?  WHITE : scene->background);
  zBuffer = initZBuffer(output->width, output->height);
  iterf(&scene->nodes, &node) {
    setCameraMat4(cameraMatrix);
    setDivideByZ(TRUE);
    drawNode(node, zBuffer, replacedNode, replacedNode == NULL ? debugMode : FALSE, camera->sceneFilterAND, camera->sceneFilterOR, output);
  }
  iterf(&camera->nodes, &node) {
    setCameraMat4(cameraMatrix);
    setDivideByZ(TRUE);
    drawNode(node, zBuffer, replacedNode, replacedNode == NULL ? debugMode : FALSE, 0x0F, 0x00, output);
  }
  free(zBuffer);
}

static void impulseNodes(Node *nodeA, Node *nodeB, float normal[3], float point[3]) {
  float temp[2][3];
  float velocity[3];
  float impulse[3];
  float impulseCoefficient;
  float impulseNumerator, impulseDenominator;
  float nodeVelocityNormal;
  float relativeVelocity[3], relativeAngVelocity[3];
  float restitution;
  restitution = sqrt(nodeA->collisionShape.restitution * nodeA->collisionShape.restitution + nodeB->collisionShape.restitution * nodeB->collisionShape.restitution);
  subVec3(nodeA->velocity, nodeB->velocity, relativeVelocity);
  subVec3(nodeA->angVelocity, nodeB->angVelocity, relativeAngVelocity);
  addVec3(relativeVelocity, cross(relativeAngVelocity, point, temp[0]), velocity);
  impulseNumerator = - (1.0F + restitution) * dot3(velocity, normal);
  if(impulseNumerator <= 0) return;
  impulseDenominator = 1.0F / nodeA->collisionShape.mass + dot3(cross(mulMat3Vec3(nodeA->collisionShape.worldInverseInertia, cross(point, normal, temp[0]), temp[1]), point, temp[0]), normal);
  impulseCoefficient = impulseNumerator / impulseDenominator;
  mulVec3ByScalar(normal, impulseCoefficient, impulse);
  divVec3ByScalar(impulse, nodeA->collisionShape.mass, temp[0]);
  nodeVelocityNormal = -dot3(nodeA->velocity, normal);
  // if(length3(temp[0]) < nodeVelocityNormal) {
  //   normalize3(temp[0], temp[1]);
  //   mulVec3ByScalar(temp[1], nodeVelocityNormal, temp[0]);
  // }
  addVec3(nodeA->velocity, temp[0], nodeA->velocity);
  addVec3(nodeA->angMomentum, cross(point, impulse, temp[0]), nodeA->angMomentum);
  mulMat3Vec3(nodeA->collisionShape.worldInverseInertia, nodeA->angMomentum, nodeA->angVelocity);
  if(length3(nodeA->velocity) > VELOCITY_LIMIT) {
		normalize3(nodeA->velocity, temp[0]);
		mulVec3ByScalar(temp[0], VELOCITY_LIMIT, nodeA->velocity);
	}
  // subVec3(relativeVelocity, mulVec3ByScalar(normal, dot3(relativeVelocity, normal), temp[0]), temp[1]);
  // normalize3(temp[1], tangent);
  // staticImpulse = - dot3(relativeVelocity, tangent) / (1.0F / nodeA->collisionShape.mass + 1.0F / nodeB->collisionShape.mass);
  // if(fabsf(staticImpulse) <= impulseCoefficient * staticFriction) {
  //   mulVec3ByScalar(tangent, staticImpulse, frictionImpulse);
  // } else {
  //   mulVec3ByScalar(tangent, - impulseCoefficient * dynamicFriction, frictionImpulse);
  // }
  // subVec3(nodeA->velocity, divVec3ByScalar(frictionImpulse, impulseDenominator, temp[0]), nodeA->velocity);
}

static void rubNodes(Node *nodeA, Node *nodeB, const float normal[3], float staticFriction, float dynamicFriction, float rollingFriction, float elapsed) {
  float temp[2][3];
  float velocityTangent[3];
  float relativeVelocity[3];
  float angVelocityNormal[3];
  subVec3(nodeA->velocity, nodeB->velocity, relativeVelocity);
  mulVec3ByScalar(mulVec3ByScalar(normal, dot3(normal, nodeA->angMomentum), temp[0]), elapsed * rollingFriction, angVelocityNormal);
  subVec3(nodeA->angMomentum, angVelocityNormal, nodeA->angMomentum);
  mulMat3Vec3(nodeA->collisionShape.worldInverseInertia, nodeA->angMomentum, nodeA->angVelocity);
  subVec3(relativeVelocity, mulVec3ByScalar(normal, dot3(relativeVelocity, normal), temp[0]), velocityTangent);
  subVec3(nodeA->velocity, mulVec3ByScalar(velocityTangent, elapsed * dynamicFriction, temp[0]), nodeA->velocity);
}

static void collideNodes(Node *nodeA, Node *nodeB, Vector *info, float staticFriction, float dynamicFriction, float rollingFriction, float elapsed) {
  float *tempPointer[1];
  int checkedFlag = FALSE;
  float *checkedNormal = NULL;
  Vector checkedNormals = initVector();
  CollisionInfoNode2Node *collisionInfo;
  if(nodeA->physicsMode == PHYSICS_3D) {
    iterf(info, &collisionInfo) {
      if(info->firstItem->data == collisionInfo) {
        float tempVec3[1][3];
        mulVec3ByScalar(collisionInfo->normal, -collisionInfo->depth, tempVec3[0]);
        addVec3(nodeA->position, tempVec3[0], nodeA->position);
        subVec3(collisionInfo->contacts[0], tempVec3[0], collisionInfo->contacts[0]);
        subVec3(collisionInfo->contacts[1], tempVec3[0], collisionInfo->contacts[1]);
      }
      impulseNodes(nodeA, nodeB, collisionInfo->normal, collisionInfo->contacts[0]);
      impulseNodes(nodeA, nodeB, collisionInfo->normal, collisionInfo->contacts[1]);
      iterf(&checkedNormals, &checkedNormal) {
        if(cosVec3(collisionInfo->normal, checkedNormal) >= 0.9F) checkedFlag = TRUE;
      }
      if(!checkedFlag) {
        tempPointer[0] = malloc(SIZE_VEC3);
        memcpy_s(tempPointer[0], SIZE_VEC3, collisionInfo->normal, SIZE_VEC3);
        rubNodes(nodeA, nodeB, tempPointer[0], staticFriction, dynamicFriction, rollingFriction, elapsed);
        push(&checkedNormals, tempPointer[0]);
      }
    }
  }
  freeVector(&checkedNormals);
}

static int is2dCollided(Scene *scene, Camera *camera, Node *node, Node *collisionTarget) {
  Scene tempScene;
  Image nodeImage, targetImage;
  Node *mainNode;
  nodeImage = initImageBulk(128, 128, NULL_COLOR);
  targetImage = initImageBulk(128, 128, NULL_COLOR);
  tempScene = *scene;
  tempScene.camera = *camera;
  tempScene.camera.aspect = 0.0F;
  tempScene.camera.parent = NULL;
  tempScene.background = WHITE;
  tempScene.camera.sceneFilterAND = 0x0F;
  tempScene.camera.sceneFilterOR = 0x00;
  mainNode = (node->aabb[0][1] > collisionTarget->aabb[0][1] && node->aabb[0][0] < collisionTarget->aabb[0][0]) ? collisionTarget : node;
  if(mainNode->parent) {
    float tempVec4[2][4];
    mulMat4Vec4(mainNode->lastTransformation, convVec3toVec4(mainNode->position, tempVec4[0]), tempVec4[1]);
    tempScene.camera.position[0] = tempVec4[1][0];
    tempScene.camera.position[1] = tempVec4[1][1];
  } else {
    tempScene.camera.position[0] = mainNode->position[0];
    tempScene.camera.position[1] = mainNode->position[1];
  }
  tempScene.camera.target[0] = tempScene.camera.position[0];
  tempScene.camera.target[1] = tempScene.camera.position[1];
  drawSceneEx(&tempScene, &nodeImage, &tempScene.camera, node);
  drawSceneEx(&tempScene, &targetImage, &tempScene.camera, collisionTarget);
  if(isImageOverlap(&nodeImage, &targetImage)) {
    freeImage(&nodeImage);
    freeImage(&targetImage);
    return TRUE;
  }
  freeImage(&nodeImage);
  freeImage(&targetImage);
  return FALSE;
}

static void runNodeBehaviour(Scene *scene, Node *node, float elapsed) {
  IntervalEventNode *interval;
  if(!node->isActive) return;
  if(node->behaviour != NULL) {
    if(!node->behaviour(node, elapsed)) return;
  }
  resetIteration(&node->intervalEvents);
  interval = nextData(&node->intervalEvents);
  while(interval) {
    interval->counter += elapsed;
    if(interval->counter >= interval->seconds) {
      interval->counter = 0.0F;
      if(!interval->callback(node, interval->data)) {
        previousData(&node->intervalEvents);
        removeByData(&node->intervalEvents, interval);
      }
    }
    interval = nextData(&node->intervalEvents);
  }
}

void updateSceneEx(Scene *scene, float rawElapsed, Camera *camera) {
  Node *node;
  NodeIter iter, cameraIter;
  IntervalEventScene *intervalScene;
  float elapsed = scene->speed * rawElapsed;
  scene->clock += elapsed;
  iter = initNodeIter(&scene->nodes);
  for(node = nextNode(&iter);node != NULL;node = nextNode(&iter)) {
    float temp[3];
    float tempMat4[4][4];
    float tempMat3[2][3][3];
    float orientation[3][3];
    CollisionInfo *info;
    if(!node->isActive) continue;
    node->previousPosition[0] = node->position[0];
    node->previousPosition[1] = node->position[1];
    if(node->collisionShape.mass != 0.0F) {
      if(node->isGravityEnabled) addVec3(node->force, mulVec3ByScalar(scene->acceleration, elapsed * node->collisionShape.mass, temp), node->force);
      addVec3(node->velocity, mulVec3ByScalar(node->force, elapsed / node->collisionShape.mass, temp), node->velocity);
      addVec3(node->velocity, divVec3ByScalar(node->impulseForce, node->collisionShape.mass, temp), node->velocity);
      addVec3(node->position, mulVec3ByScalar(node->velocity, elapsed, temp), node->position);
      genRotationMat4(node->angle[0], node->angle[1], node->angle[2], tempMat4);
      convMat4toMat3(tempMat4, orientation);
      genSkewMat3(node->angVelocity, tempMat3[0]);
      addMat3(orientation, mulMat3ByScalar(mulMat3(tempMat3[0], orientation, tempMat3[1]), elapsed, tempMat3[0]), tempMat3[1]);
      orthogonalize3(tempMat3[1], orientation);
      getAngleFromMat3(orientation, node->angle);
      addVec3(node->angMomentum, mulVec3ByScalar(node->torque, elapsed, temp), node->angMomentum);
      mulMat3(orientation, mulMat3(node->collisionShape.inverseInertia, transposeMat3(orientation, tempMat3[0]), tempMat3[1]), node->collisionShape.worldInverseInertia);
      mulMat3Vec3(node->collisionShape.worldInverseInertia, node->angMomentum, node->angVelocity);
    }
    subVec3(node->force, mulVec3ByScalar(node->force, elapsed, temp), node->force);
    clearVec3(node->impulseForce);
    clearVec3(node->torque);
    getShapeAABB(node->collisionShape, getWorldTransfomration(node, tempMat4), node->aabb);
    iterf(&node->collisionTargets, &info) freeVector(&info->info);
    freeVector(&node->collisionTargets);
    node->collisionFlags = 0;
  }
  for(node = nextNode(&iter);node != NULL;node = nextNode(&iter)) {
    if(node->collisionMaskActive || node->collisionMaskPassive) {
      Node *collisionTarget;
      NodeIter targetIter;
      if(!node->isActive) continue;
      targetIter.iterStack = initVector();
      concatVectorAlloc(&targetIter.iterStack, &iter.iterStack, sizeof(VectorIter));
      targetIter.currentIter = iter.currentIter;
      targetIter.currentNode = iter.currentNode;
      for(collisionTarget = nextNode(&targetIter);collisionTarget != NULL;collisionTarget = nextNode(&targetIter)) {
        unsigned int flagsA = node->collisionMaskPassive & collisionTarget->collisionMaskActive;
        unsigned int flagsB = node->collisionMaskActive & collisionTarget->collisionMaskPassive;
        if(!collisionTarget->isActive) continue;
        if(flagsA | flagsB) {
          if(testCollision(*node, *collisionTarget)) {
            if(node->physicsMode == PHYSICS_2D && collisionTarget->physicsMode == PHYSICS_2D) {
              if(is2dCollided(scene, camera, node, collisionTarget)) {
                CollisionInfo userInfo = { 0 };
                if(!(node->isThrough || collisionTarget->isThrough)) {
                  node->position[0] = node->previousPosition[0];
                  node->position[1] = node->previousPosition[1];
                  collisionTarget->position[0] = collisionTarget->previousPosition[0];
                  collisionTarget->position[1] = collisionTarget->previousPosition[1];
                  clearVec2(node->velocity);
                  clearVec2(collisionTarget->velocity);
                }
                userInfo.target = collisionTarget;
                pushAlloc(&node->collisionTargets, sizeof(CollisionInfo), &userInfo);
                userInfo.target = node;
                pushAlloc(&collisionTarget->collisionTargets, sizeof(CollisionInfo), &userInfo);
                node->collisionFlags |= flagsA;
                collisionTarget->collisionFlags |= flagsB;
              }
            } else {
              Vector infoA, infoB;
              if(testCollisionPolygonPolygon(*node, *collisionTarget, &infoA, &infoB)) {
                CollisionInfo userInfo;
                if(!(node->isThrough || collisionTarget->isThrough)) {
                  float staticFriction, dynamicFriction, rollingFriction;
                  staticFriction = sqrtf(node->collisionShape.staticFriction * node->collisionShape.staticFriction + collisionTarget->collisionShape.staticFriction * collisionTarget->collisionShape.staticFriction);
                  dynamicFriction = sqrtf(node->collisionShape.dynamicFriction * node->collisionShape.dynamicFriction + collisionTarget->collisionShape.dynamicFriction * collisionTarget->collisionShape.dynamicFriction);
                  rollingFriction = sqrtf(node->collisionShape.rollingFriction * node->collisionShape.rollingFriction + collisionTarget->collisionShape.rollingFriction * collisionTarget->collisionShape.rollingFriction);
                  collideNodes(node, collisionTarget, &infoA, staticFriction, dynamicFriction, rollingFriction, elapsed);
                  collideNodes(collisionTarget, node, &infoB, staticFriction, dynamicFriction, rollingFriction, elapsed);
                }
                userInfo.target = collisionTarget;
                userInfo.info = infoA;
                pushAlloc(&node->collisionTargets, sizeof(CollisionInfo), &userInfo);
                userInfo.target = node;
                userInfo.info = infoB;
                pushAlloc(&collisionTarget->collisionTargets, sizeof(CollisionInfo), &userInfo);
                node->collisionFlags |= flagsA;
                collisionTarget->collisionFlags |= flagsB;
              }
            }
          }
        }
      }
    }
  }
  for(node = nextNode(&iter);node != NULL;node = nextNode(&iter)) runNodeBehaviour(scene, node, elapsed);
  cameraIter = initNodeIter(&camera->nodes);
  for(node = nextNode(&cameraIter);node != NULL;node = nextNode(&cameraIter)) runNodeBehaviour(scene, node, elapsed);
  if(scene->behaviour) scene->behaviour(scene, elapsed);
  resetIteration(&scene->intervalEvents);
  intervalScene = nextData(&scene->intervalEvents);
  while(intervalScene) {
    intervalScene->counter += elapsed;
    if(intervalScene->counter >= intervalScene->seconds) {
      intervalScene->counter = 0.0F;
      if(!intervalScene->callback(scene, intervalScene->data)) {
        previousData(&scene->intervalEvents);
        removeByData(&scene->intervalEvents, intervalScene);
      }
    }
    intervalScene = nextData(&scene->intervalEvents);
  }
}

void discardScene(Scene *scene) {
  clearVector(&scene->nodes);
}
