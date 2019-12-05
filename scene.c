#include<Windows.h>
#include<time.h>
#include<math.h>

#include "./include/borland.h"
#include "./include/scene.h"
#include "./include/node.h"
#include "./include/vector.h"
#include "./include/matrix.h"
#include "./include/graphics.h"

#define VELOCITY_LIMIT 200.0F * 10000.0F / 3600.0F

Camera initCamera(float x, float y, float z, float aspect) {
  Camera camera;
  memset(&camera, 0, sizeof(Camera));
  camera.position[0] = x;
  camera.position[1] = y;
  camera.position[2] = z;
  camera.worldUp[0] = 0.0F;
  camera.worldUp[1] = 1.0F;
  camera.worldUp[2] = 0.0F;
  camera.fov = PI / 3.0F * 2.0F;
  camera.nearLimit = 10.0F;
  camera.farLimit = 1000.0F;
  camera.aspect = aspect;
  return camera;
}

Scene initScene(void) {
  Scene scene;
  memset(&scene, 0, sizeof(Scene));
  scene.acceleration[1] = -98.0F;
  scene.nodes = initVector();
  scene.camera = initCamera(0.0F, 0.0F, 0.0F, 0.0F);
  return scene;
}

void addIntervalEventScene(Scene *scene, unsigned int milliseconds, void (*callback)(Scene*)) {
  IntervalEventScene *interval = malloc(sizeof(IntervalEventScene));
  interval->begin = clock();
  interval->interval = milliseconds * CLOCKS_PER_SEC / 1000;
  interval->callback = callback;
  push(&scene->intervalEvents, interval);
}

void drawSceneWithCamera(Scene *scene, Image *output, Camera *camera) {
  Node *node;
  float lookAt[4][4];
  float projection[4][4];
  float cameraMatrix[4][4];
  float *zBuffer;
  float aspect;
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
  clearImage(output, scene->background);
  zBuffer = initZBuffer(output->width, output->height);
  iterf(&scene->nodes, &node) {
    setCameraMat4(cameraMatrix);
    drawNode(node, zBuffer, output);
  }
}

void drawScene(Scene *scene, Image *output) {
  drawSceneWithCamera(scene, output, &scene->camera);
}

static void impulseNodes(Node *nodeA, Node *nodeB, float normal[3], float point[3], float depth) {
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
  impulseCoefficient = impulseNumerator / impulseDenominator - 2.0F * depth;
  mulVec3ByScalar(normal, impulseCoefficient, impulse);
  divVec3ByScalar(impulse, nodeA->collisionShape.mass, temp[0]);
  nodeVelocityNormal = -dot3(nodeA->velocity, normal);
  if(length3(temp[0]) < nodeVelocityNormal) {
    normalize3(temp[0], temp[1]);
    mulVec3ByScalar(temp[1], nodeVelocityNormal, temp[0]);
  }
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
  if(nodeA->isPhysicsEnabled) {
    iterf(info, &collisionInfo) {
      impulseNodes(nodeA, nodeB, collisionInfo->normal, collisionInfo->contacts[0], collisionInfo->depth);
      impulseNodes(nodeA, nodeB, collisionInfo->normal, collisionInfo->contacts[1], collisionInfo->depth);
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

void updateScene(Scene *scene, float elapsed) {
  Node *node;
  NodeIter iter;
  IntervalEventScene *intervalScene;
  scene->clock += elapsed;
  iter = initNodeIter(&scene->nodes);
  for(node = nextNode(&iter);node != NULL;node = nextNode(&iter)) {
    float temp[3];
    float tempMat4[4][4];
    float tempMat3[2][3][3];
    float orientation[3][3];
    CollisionInfo *info;
    if(node->collisionShape.mass == 0.0F) continue;
    if(node->isPhysicsEnabled) addVec3(node->force, mulVec3ByScalar(scene->acceleration, node->collisionShape.mass, temp), node->force);
    addVec3(node->position, mulVec3ByScalar(node->velocity, elapsed, temp), node->position);
    genRotationMat4(node->angle[0], node->angle[1], node->angle[2], tempMat4);
    convMat4toMat3(tempMat4, orientation);
    genSkewMat3(node->angVelocity, tempMat3[0]);
    addMat3(orientation, mulMat3ByScalar(mulMat3(tempMat3[0], orientation, tempMat3[1]), elapsed, tempMat3[0]), tempMat3[1]);
    orthogonalize3(tempMat3[1], orientation);
    getAngleFromMat3(orientation, node->angle);
    addVec3(node->velocity, mulVec3ByScalar(node->force, elapsed / node->collisionShape.mass, temp), node->velocity);
    addVec3(node->angMomentum, mulVec3ByScalar(node->torque, elapsed, temp), node->angMomentum);
    mulMat3(orientation, mulMat3(node->collisionShape.inverseInertia, transposeMat3(orientation, tempMat3[0]), tempMat3[1]), node->collisionShape.worldInverseInertia);
    mulMat3Vec3(node->collisionShape.worldInverseInertia, node->angMomentum, node->angVelocity);
    iterf(&node->collisionTargets, &info) freeVector(&info->info);
    freeVector(&node->collisionTargets);
    clearVec3(node->force);
    clearVec3(node->torque);
    node->collisionFlags = 0;
  }
  for(node = nextNode(&iter);node != NULL;node = nextNode(&iter)) {
    if(node->collisionMaskActive || node->collisionMaskPassive) {
      Node nodeTemp;
      Node *collisionTarget;
      NodeIter targetIter;
      nodeTemp = *node;
      targetIter.iterStack = initVector();
      concatVectorAlloc(&targetIter.iterStack, &iter.iterStack, sizeof(VectorIter));
      targetIter.currentIter = iter.currentIter;
      targetIter.currentNode = iter.currentNode;
      for(collisionTarget = nextNode(&targetIter);collisionTarget != NULL;collisionTarget = nextNode(&targetIter)) {
        unsigned int flagsA = node->collisionMaskPassive & collisionTarget->collisionMaskActive;
        unsigned int flagsB = node->collisionMaskActive & collisionTarget->collisionMaskPassive;
        if(flagsA | flagsB) {
          if(testCollision(*node, *collisionTarget)) {
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
  for(node = nextNode(&iter);node != NULL;node = nextNode(&iter)) {
    IntervalEventNode *interval;
    clearVec3(node->force);
    clearVec3(node->torque);
    if(node->behaviour != NULL) {
      if(!node->behaviour(node)) {
        node = previousData(&scene->nodes);
        continue;
      }
    }
    resetIteration(&node->intervalEvents);
    interval = nextData(&node->intervalEvents);
    while(interval) {
      clock_t current = clock();
      clock_t diff = current - interval->begin;
      if(diff < 0) {
        interval->begin = current;
      } else {
        if(interval->interval < (unsigned int)diff) {
          interval->begin = current;
          interval->callback(node);
        }
      }
      interval = nextData(&node->intervalEvents);
    }
  }
  if(scene->behaviour) scene->behaviour(scene, elapsed);
  resetIteration(&scene->intervalEvents);
  intervalScene = nextData(&scene->intervalEvents);
  while(intervalScene) {
    clock_t current = clock();
    clock_t diff = current - intervalScene->begin;
    if(diff < 0) {
      intervalScene->begin = current;
    } else {
      if(intervalScene->interval < (unsigned int)diff) {
        intervalScene->begin = current;
        intervalScene->callback(scene);
      }
    }
    intervalScene = nextData(&scene->intervalEvents);
  }
}

void discardScene(Scene *scene) {
  clearVector(&scene->nodes);
}
