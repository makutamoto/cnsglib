#include<Windows.h>
#include<time.h>
#include<math.h>

#include "./include/borland.h"
#include "./include/scene.h"
#include "./include/node.h"
#include "./include/vector.h"
#include "./include/matrix.h"
#include "./include/graphics.h"

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
  camera.farLimit = 500.0F;
  camera.aspect = aspect;
  return camera;
}

Scene initScene(void) {
  Scene scene;
  memset(&scene, 0, sizeof(Scene));
  scene.acceleration[1] = -98.0F;
  scene.nodes = initVector();
  scene.camera = initCamera(0.0F, 0.0F, 0.0F, 1.0F);
  return scene;
}

void addIntervalEventScene(Scene *scene, unsigned int milliseconds, void (*callback)(Scene*)) {
  IntervalEventScene *interval = malloc(sizeof(IntervalEventScene));
  interval->begin = clock();
  interval->interval = milliseconds * CLOCKS_PER_SEC / 1000;
  interval->callback = callback;
  push(&scene->intervalEvents, interval);
}

void drawScene(Scene *scene) {
  Node *node;
  float lookAt[4][4];
  float projection[4][4];
  float camera[4][4];
  clearTransformation();
  if(scene->camera.parent) {
    float temp[4];
    float cameraParent[4][4];
    float cameraPosition[4];
    float cameraTarget[4];
    getWorldTransfomration(*scene->camera.parent, cameraParent);
    mulMat4Vec4(cameraParent, convVec3toVec4(scene->camera.position, temp), cameraPosition);
    if(scene->camera.positionMask[0]) cameraPosition[0] = scene->camera.parent->position[0] + scene->camera.position[0];
    if(scene->camera.positionMask[1]) cameraPosition[1] = scene->camera.parent->position[1] + scene->camera.position[1];
    if(scene->camera.positionMask[2]) cameraPosition[2] = scene->camera.parent->position[2] + scene->camera.position[2];
    mulMat4Vec4(cameraParent, convVec3toVec4(scene->camera.target, temp), cameraTarget);
    genLookAtMat4(cameraPosition, cameraTarget, scene->camera.worldUp, lookAt);
  } else {
    genLookAtMat4(scene->camera.position, scene->camera.target, scene->camera.worldUp, lookAt);
  }
  genPerspectiveMat4(scene->camera.fov, scene->camera.nearLimit, scene->camera.farLimit, scene->camera.aspect, projection);
  mulMat4(projection, lookAt, camera);
  setZNear(scene->camera.nearLimit);
  clearBuffer(scene->background);
  clearZBuffer();
  iterf(&scene->nodes, &node) {
    setCameraMat4(camera);
    drawNode(node);
    clearVector(&node->collisionTargets);
    node->collisionFlags = 0;
  }
  flushBuffer();
}

static void impulseNodes(Node *nodeA, Node *nodeB, float normal[3], float point[3]) {
  float temp[2][3];
  float velocity[3];
  float impulse[3];
  float impulseCoefficient;
  float impulseNumerator, impulseDenominator;
  addVec3(nodeA->velocity, cross(nodeA->angVelocity, point, temp[0]), velocity);
  impulseNumerator = - (1.0F + nodeA->shape.restitution) * dot3(velocity, normal);
  if(impulseNumerator <= 0) return;
  impulseDenominator = 1.0F / nodeA->shape.mass + dot3(cross(mulMat3Vec3(nodeA->shape.worldInverseInertia, cross(point, normal, temp[0]), temp[1]), point, temp[0]), normal);
  impulseCoefficient = impulseNumerator / impulseDenominator;
  mulVec3ByScalar(normal, impulseCoefficient, impulse);
  addVec3(nodeA->velocity, divVec3ByScalar(impulse, nodeA->shape.mass, temp[0]), nodeA->velocity);
  addVec3(nodeA->angMomentum, cross(point, impulse, temp[0]), nodeA->angMomentum);
  mulMat3Vec3(nodeA->shape.worldInverseInertia, nodeA->angMomentum, nodeA->angVelocity);
  // subVec3(relativeVelocity, mulVec3ByScalar(normal, dot3(relativeVelocity, normal), temp[0]), temp[1]);
  // normalize3(temp[1], tangent);
  // staticImpulse = - dot3(relativeVelocity, tangent) / (1.0F / nodeA->shape.mass + 1.0F / nodeB->shape.mass);
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
  mulMat3Vec3(nodeA->shape.worldInverseInertia, nodeA->angMomentum, nodeA->angVelocity);
  subVec3(relativeVelocity, mulVec3ByScalar(normal, dot3(relativeVelocity, normal), temp[0]), velocityTangent);
  subVec3(nodeA->velocity, mulVec3ByScalar(velocityTangent, elapsed * dynamicFriction, temp[0]), nodeA->velocity);
}

void updateScene(Scene *scene, float elapsed) {
  Node *node;
  IntervalEventScene *intervalScene;
  resetIteration(&scene->nodes);
  node = nextData(&scene->nodes);
  while(node) {
    float temp[3];
    float tempMat4[4][4];
    float tempMat3[2][3][3];
    float orientation[3][3];
    if(node->isPhysicsEnabled) addVec3(node->force, mulVec3ByScalar(scene->acceleration, node->shape.mass, temp), node->force);
    addVec3(node->position, mulVec3ByScalar(node->velocity, elapsed, temp), node->position);
    genRotationMat4(node->angle[0], node->angle[1], node->angle[2], tempMat4);
    convMat4toMat3(tempMat4, orientation);
    genSkewMat3(node->angVelocity, tempMat3[0]);
    addMat3(orientation, mulMat3ByScalar(mulMat3(tempMat3[0], orientation, tempMat3[1]), elapsed, tempMat3[0]), tempMat3[1]);
    orthogonalize3(tempMat3[1], orientation);
    getAngleFromMat3(orientation, node->angle);
    addVec3(node->velocity, mulVec3ByScalar(node->force, elapsed / node->shape.mass, temp), node->velocity);
    addVec3(node->angMomentum, mulVec3ByScalar(node->torque, elapsed, temp), node->angMomentum);
    mulMat3(orientation, mulMat3(node->shape.inverseInertia, transposeMat3(orientation, tempMat3[0]), tempMat3[1]), node->shape.worldInverseInertia);
    mulMat3Vec3(node->shape.worldInverseInertia, node->angMomentum, node->angVelocity);
    node = nextData(&scene->nodes);
  }
  iterf(&scene->nodes, &node) {
    if(node->collisionMaskActive || node->collisionMaskPassive) {
      Node nodeTemp;
      Node *collisionTarget;
      VectorItem *item = scene->nodes.currentItem;
      nodeTemp = *node;
      collisionTarget = (Node*)nextData(&scene->nodes);
      while(collisionTarget) {
        unsigned int flagsA = node->collisionMaskPassive & collisionTarget->collisionMaskActive;
        unsigned int flagsB = node->collisionMaskActive & collisionTarget->collisionMaskPassive;
        unsigned int flags = flagsA | flagsB;
        if(flags) {
          if(testCollision(*node, *collisionTarget)) {
            Vector points = initVector();
            Vector normals = initVector();
            if(testCollisionPolygonPolygon(*node, *collisionTarget, &normals, &points)) {
              if(!node->isThrough) {
                float tempVec3[1][3];
                float *point, (*normal)[3];
                float staticFriction, dynamicFriction, rollingFriction;
                Vector checkedNormalsNode = initVector();
                Vector checkedNormalsTarget = initVector();
                staticFriction = sqrtf(node->shape.staticFriction * node->shape.staticFriction + collisionTarget->shape.staticFriction * collisionTarget->shape.staticFriction);
                dynamicFriction = sqrtf(node->shape.dynamicFriction * node->shape.dynamicFriction + collisionTarget->shape.dynamicFriction * collisionTarget->shape.dynamicFriction);
                rollingFriction = sqrtf(node->shape.rollingFriction * node->shape.rollingFriction + collisionTarget->shape.rollingFriction * collisionTarget->shape.rollingFriction);
                resetIteration(&normals);
                normal = nextData(&normals);
                iterf(&points, &point) {
                  int checkedFlag = FALSE;
                  float *checkedNormal = NULL;
                  if(node->isPhysicsEnabled) {
                    subVec3(point, node->position, tempVec3[0]);
                    impulseNodes(node, collisionTarget, normal[1], tempVec3[0]);
                    iterf(&checkedNormalsTarget, &checkedNormal) {
                      if(cosVec3(normal[1], checkedNormal) >= 0.9F) checkedFlag = TRUE;
                    }
                    if(!checkedFlag) {
                      float *temp = malloc(SIZE_VEC3);
                      memcpy_s(temp, SIZE_VEC3, normal[1], SIZE_VEC3);
                      rubNodes(node, collisionTarget, temp, staticFriction, dynamicFriction, rollingFriction, elapsed);
                      push(&checkedNormalsTarget, temp);
                    }
                  }
                  if(collisionTarget->isPhysicsEnabled) {
                    subVec3(point, collisionTarget->position, tempVec3[0]);
                    impulseNodes(collisionTarget, &nodeTemp, normal[0], tempVec3[0]);
                    iterf(&checkedNormalsNode, &checkedNormal) {
                      if(cosVec3(normal[0], checkedNormal) >= 0.9F) checkedFlag = TRUE;
                    }
                    if(!checkedFlag) {
                      float *temp = malloc(SIZE_VEC3);
                      memcpy_s(temp, SIZE_VEC3, normal[0], SIZE_VEC3);
                      rubNodes(collisionTarget, &nodeTemp, temp, staticFriction, dynamicFriction, rollingFriction, elapsed);
                      push(&checkedNormalsNode, temp);
                    }
                  }
                  normal = nextData(&normals);
                }
              }
              push(&node->collisionTargets, collisionTarget);
              push(&collisionTarget->collisionTargets, node);
              node->collisionFlags |= flags;
              collisionTarget->collisionFlags |= flags;
            }
          }
        }
        collisionTarget = (Node*)nextData(&scene->nodes);
      }
      scene->nodes.currentItem = item;
    }
  }
  resetIteration(&scene->nodes);
  node = previousData(&scene->nodes);
  while(node) {
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
    node = previousData(&scene->nodes);
  }
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
