#include<stdio.h>
#include<math.h>
#include<time.h>
#include<Windows.h>

#include "./include/node.h"
#include "./include/borland.h"
#include "./include/graphics.h"
#include "./include/vector.h"
#include "./include/matrix.h"
#include "./include/colors.h"
#include "./include/triangle.h"

#define OBJ_LINE_BUFFER_SIZE 128
#define OBJ_WORD_BUFFER_SIZE 32

Node initNode(const char *id, Image image) {
  Node node;
  memset(&node, 0, sizeof(Node));
  strcpy_s(node.id, sizeof(node.id), id);
	node.scale[0] = 1.0;
	node.scale[1] = 1.0;
  node.scale[2] = 1.0;
	node.texture = image;
  node.colorFilterAND = 0x0F;
  node.children = initVector();
  node.isActive = TRUE;
  node.isVisible = TRUE;
  genIdentityMat4(node.lastTransformation);
  return node;
}

Node initNodeUI(const char *id, Image image, unsigned char color) {
  Node node;
  node = initNode(id, image);
  node.shape = initShapePlaneInv(1.0F, 1.0F, color);
  node.isInterface = TRUE;
  return node;
}

Node initNodeSprite(const char *id, float width, float height, Image texture, Image collisionTexture) {
  Node node;
  node = initNode(id, texture);
  node.collisionTexture = collisionTexture;
  node.shape = initShapePlaneV(width, height, MAGENTA);
  node.collisionShape = node.shape;
  node.physicsMode = PHYSICS_2D;
  return node;
}

Node initNodeText(const char *id, float px, float py, Align alignX, Align alignY, unsigned int sx, unsigned int sy, int (*behaviour)(Node*, float)) {
  Node node;
  node = initNodeUI(id, initImage(sx, sy, BLACK, BLACK), NULL_COLOR);
  node.position[0] = px;
  node.position[1] = py;
	node.scale[0] = sx;
	node.scale[1] = sy;
  node.behaviour = behaviour;
  node.interfaceAlign[0] = alignX;
  node.interfaceAlign[1] = alignY;
  return node;
}

NodeIter initNodeIter(Vector *layer) {
  NodeIter iter;
  iter.iterStack = initVector();
  iter.currentIter = initVectorIter(layer);
  iter.currentNode = NULL;
  return iter;
}

Node* nextNode(NodeIter *iter) {
  if(iter->currentNode == NULL) {
    iter->currentNode = nextDataIter(&iter->currentIter);
  } else {
    if(iter->currentNode->children.length != 0) {
      pushAlloc(&iter->iterStack, sizeof(VectorIter), &iter->currentIter);
      iter->currentIter = initVectorIter(&iter->currentNode->children);
      iter->currentNode = nextDataIter(&iter->currentIter);
    } else {
      iter->currentNode = nextDataIter(&iter->currentIter);
      if(iter->currentNode == NULL) {
        VectorIter *iterTemp = pop(&iter->iterStack);
        if(iterTemp == NULL) return NULL;
        iter->currentIter = *iterTemp;
        free(iterTemp);
        iter->currentNode = nextDataIter(&iter->currentIter);
      }
    }
  }
  return iter->currentNode;
}

void addNodeChild(Node *parent, Node *child) {
  push(&parent->children, child);
  child->parent = parent;
}

void freeNodeIntervals(Node *node) {
  IntervalEventNode *interval;
  interval = pop(&node->intervalEvents);
  while(interval) {
    if(interval->data) free(interval->data);
    interval = pop(&node->intervalEvents);
  }
}

void discardNode(Node *node) {
  if(node->parent) removeByData(&node->parent->children, node);
  clearVector(&node->children);
}

void discardSprite(Node *node) {
  discardShape(node->shape);
  discardNode(node);
}

void drawNode(Node *node, float zBuffer[], Node *replacedNode, unsigned char filterAND, unsigned char filterOR, Image *output) {
  Node *child;
  unsigned int halfWidth, halfHeight;
  if(!node->isActive) return;
  halfWidth = output->width / 2;
  halfHeight = output->height / 2;
	pushTransformation();
  if(node->isInterface) {
    float x, y;
    float halfScale[2];
    halfScale[0] = node->scale[0] / 2.0F;
    halfScale[1] = node->scale[1] / 2.0F;
    switch(node->interfaceAlign[0]) {
      case LEFT:
      case TOP:
        x = node->position[0] + halfScale[0];
        break;
      case CENTER:
        x = halfWidth + node->position[0];
        break;
      case RIGHT:
      case BOTTOM:
        x = output->width - node->position[0] - halfScale[0];
        break;
    }
    switch(node->interfaceAlign[1]) {
      case TOP:
      case LEFT:
        y = node->position[1] + halfScale[1];
        break;
      case CENTER:
        y = halfHeight + node->position[1];
        break;
      case BOTTOM:
      case RIGHT:
        y = output->height - node->position[1] - halfScale[1];
        break;
    }
    translateTransformation(x / halfWidth - 1.0F, y / halfHeight - 1.0F, node->position[2] > 0 ? 1.0F + node->position[2] : 1.0F);
  } else {
    translateTransformation(node->position[0], node->position[1], node->position[2]);
  }
  rotateTransformation(node->angle[0], node->angle[1], node->angle[2]);
  pushTransformation();
  if(node->isInterface) {
    clearCameraMat4();
    setDivideByZ(FALSE);
    setColorFilterAND(0x0F & node->colorFilterAND);
    setColorFilterOR(0x00 | node->colorFilterOR);
    scaleTransformation(node->scale[0] / halfWidth, node->scale[1] / halfHeight, 1.0F);
  } else {
    setColorFilterAND(filterAND & node->colorFilterAND);
    setColorFilterOR(filterOR | node->colorFilterOR);
    scaleTransformation(node->scale[0], node->scale[1], node->scale[2]);
  }
  getTransformation(node->lastTransformation);
  setFakeZ(node->useFakeZ, node->fakeZ);
  if(replacedNode) {
    if(node == replacedNode) {
      fillPolygons(node->shape.vertices, node->shape.indices, node->collisionTexture, node->shape.uv, node->shape.uvIndices, zBuffer, output);
    }
  } else {
    if(node->isVisible) {
      clearAABB();
      fillPolygons(node->shape.vertices, node->shape.indices, node->texture, node->shape.uv, node->shape.uvIndices, zBuffer, output);
      getAABB(node->aabb);
    } else {
      getShapeAABB(node->shape, node->lastTransformation, node->aabb);
    }
  }
  popTransformation();
  resetIteration(&node->children);
  child = previousData(&node->children);
  while(child) {
    drawNode(child, zBuffer, replacedNode, filterAND, filterOR, output);
    child = previousData(&node->children);
  }
  popTransformation();
}

void applyForce(Node *node, float force[3], int mask, int rotation) {
  float temp[3];
  float tempMat4[4][4];
  float tempMat3[3][3];
  if(rotation) {
    genRotationMat4(node->angle[0], node->angle[1], node->angle[2], tempMat4);
    mulMat3Vec3(convMat4toMat3(tempMat4, tempMat3), force, temp);
  } else {
    memcpy_s(temp, SIZE_VEC3, force, SIZE_VEC3);
  }
  if(mask & X_MASK) node->force[0] += temp[0];
  if(mask & Y_MASK) node->force[1] += temp[1];
  if(mask & Z_MASK) node->force[2] += temp[2];
}

float (*getNodeTransformation(Node node, float out[4][4]))[4] {
  float temp[2][4][4];
  genTranslationMat4(node.position[0], node.position[1], node.position[2], temp[0]);
  genRotationMat4(node.angle[0], node.angle[1], node.angle[2], temp[1]);
  mulMat4(temp[0], temp[1], out);
  return out;
}

float (*getWorldTransfomration(Node *node, float out[4][4]))[4] {
  Node *current = node;
  genIdentityMat4(out);
  while(current) {
    float temp[2][4][4];
    memcpy_s(temp[0], sizeof(temp[0]), out, sizeof(temp[0]));
    getNodeTransformation(*current, temp[1]);
    mulMat4(temp[1], temp[0], out);
    current = current->parent;
  }
  return out;
}

int testCollision(Node a, Node b) {
  return (a.aabb[0][0] <= b.aabb[0][1] && a.aabb[0][1] >= b.aabb[0][0]) &&
         (a.aabb[1][0] <= b.aabb[1][1] && a.aabb[1][1] >= b.aabb[1][0]) &&
         (a.aabb[2][0] <= b.aabb[2][1] && a.aabb[2][1] >= b.aabb[2][0]);
}

float (*mulMat4ByTriangle(float mat[4][4], float triangle[3][3], float out[3][3]))[3] {
  int i;
  for(i = 0;i < 3;i++) {
    float point[4];
    float transformed[4];
    copyVec3(point, triangle[i]);
    point[3] = 1.0F;
    mulMat4Vec4(mat, point, transformed);
    out[i][0] = transformed[0];
    out[i][1] = transformed[1];
    out[i][2] = transformed[2];
  }
  return out;
}

Vector* getPolygons(Node node, Vector *polygons) {
	size_t i1, i2;
	resetIteration(&node.collisionShape.indices);
	for(i1 = 0;i1 < node.collisionShape.indices.length / 3;i1++) {
    float *triangle = malloc(9 * sizeof(float));
    for(i2 = 0;i2 < 3;i2++) {
			unsigned long index = *(unsigned long*)nextData(&node.collisionShape.indices);
      float *data = dataAt(&node.collisionShape.vertices, index);
      triangle[i2 * 3] = data[0];
      triangle[i2 * 3 + 1] = data[1];
      triangle[i2 * 3 + 2] = data[2];
		}
    push(polygons, triangle);
	}
  return polygons;
}

CollisionInfoNode2Node initCollisionInfoNode2Node(Node *nodeA, Node *nodeB, float triangle[3][3], unsigned long normalIndex, unsigned long *uvIndex[3], float contacts[2][3], float depth) {
  float tempVec3[2][3];
  float tempVec4[2][4];
  float tempMat3[2][3][3];
  float tempMat4[1][4][4];
  CollisionInfoNode2Node info;
  memcpy_s(info.normal, SIZE_VEC3, dataAt(&nodeB->collisionShape.normals, normalIndex), SIZE_VEC3);
  transposeMat3(inverse3(convMat4toMat3(nodeB->lastTransformation, tempMat3[0]), tempMat3[1]), tempMat3[0]);
  mulMat3Vec3(tempMat3[0], info.normal, tempVec3[0]);
  normalize3(tempVec3[0], info.normal);
  convVec4toVec3(mulMat4Vec4(getWorldTransfomration(nodeA->parent, tempMat4[0]), convVec3toVec4(nodeA->position, tempVec4[0]), tempVec4[1]), tempVec3[0]);
  subVec3(contacts[0], tempVec3[0], info.contacts[0]);
  subVec3(contacts[1], tempVec3[0], info.contacts[1]);
  cartesian3dToBarycentric(triangle, contacts[0], info.barycentric[0]);
  cartesian3dToBarycentric(triangle, contacts[1], info.barycentric[1]);
  info.depth = depth;
  return info;
}

int testCollisionPolygonPolygon(Node a, Node b, Vector *infoAOut, Vector *infoBOut) {
  unsigned long *normalIndex[2];
  unsigned long *uvIndex[2][3];
  Vector polygonsA = initVector();
  Vector polygonsB = initVector();
  float (*polygonA)[3], (*polygonB)[3];
  int collided = FALSE;
  *infoAOut = initVector();
  *infoBOut = initVector();
  resetIteration(&a.collisionShape.normalIndices);
  normalIndex[0] = nextData(&a.collisionShape.normalIndices);
  resetIteration(&a.collisionShape.uvIndices);
  uvIndex[0][0] = nextData(&a.collisionShape.uvIndices);
  uvIndex[0][1] = nextData(&a.collisionShape.uvIndices);
  uvIndex[0][2] = nextData(&a.collisionShape.uvIndices);
  getPolygons(a, &polygonsA);
  getPolygons(b, &polygonsB);
  polygonA = nextData(&polygonsA);
  while(polygonA) {
    float polygonAWorld[3][3];
    resetIteration(&b.collisionShape.normalIndices);
    normalIndex[1] = nextData(&b.collisionShape.normalIndices);
    resetIteration(&b.collisionShape.uvIndices);
    uvIndex[1][0] = nextData(&b.collisionShape.uvIndices);
    uvIndex[1][1] = nextData(&b.collisionShape.uvIndices);
    uvIndex[1][2] = nextData(&b.collisionShape.uvIndices);
    mulMat4ByTriangle(a.lastTransformation, polygonA, polygonAWorld);
    resetIteration(&polygonsB);
    polygonB = nextData(&polygonsB);
    while(polygonB) {
      float polygonBWorld[3][3];
      float contacts[2][3];
      float depths[2];
      mulMat4ByTriangle(b.lastTransformation, polygonB, polygonBWorld);
      if(testCollisionTriangleTriangle(polygonAWorld, polygonBWorld, contacts, depths)) {
        CollisionInfoNode2Node infoA, infoB;
        infoA = initCollisionInfoNode2Node(&a, &b, polygonA, *normalIndex[1], uvIndex[1], contacts, depths[0]);
        infoB = initCollisionInfoNode2Node(&b, &a, polygonB, *normalIndex[0], uvIndex[0], contacts, depths[1]);
        pushAlloc(infoAOut, sizeof(CollisionInfoNode2Node), &infoA);
        pushAlloc(infoBOut, sizeof(CollisionInfoNode2Node), &infoB);
        collided = TRUE;
      }
      polygonB = nextData(&polygonsB);
      normalIndex[1] = nextData(&b.collisionShape.normalIndices);
      uvIndex[1][0] = nextData(&b.collisionShape.uvIndices);
      uvIndex[1][1] = nextData(&b.collisionShape.uvIndices);
      uvIndex[1][2] = nextData(&b.collisionShape.uvIndices);
    }
    polygonA = nextData(&polygonsA);
    normalIndex[0] = nextData(&a.collisionShape.normalIndices);
    uvIndex[0][0] = nextData(&a.collisionShape.uvIndices);
    uvIndex[0][1] = nextData(&a.collisionShape.uvIndices);
    uvIndex[0][2] = nextData(&a.collisionShape.uvIndices);
  }
  freeVector(&polygonsA);
  freeVector(&polygonsB);
  return collided;
}

void addIntervalEventNode(Node *node, unsigned int milliseconds, int (*callback)(Node*, void*), void *data) {
  IntervalEventNode *interval = malloc(sizeof(IntervalEventNode));
  interval->begin = clock();
  interval->interval = milliseconds * CLOCKS_PER_SEC / 1000;
  interval->data = data;
  interval->callback = callback;
  push(&node->intervalEvents, interval);
}

Shape initShape(float mass) {
  Shape shape;
  memset(&shape, 0, sizeof(Shape));
  shape.indices = initVector();
  shape.vertices = initVector();
  shape.normals = initVector();
  shape.normalIndices = initVector();
  shape.uv = initVector();
  shape.uvIndices = initVector();
  shape.mass = mass;
  return shape;
}

Shape initShapePlane(float width, float height, unsigned char color, float mass) {
  int i;
  float halfWidth = width / 2.0F;
  float halfHeight = height / 2.0F;
  Shape shape = initShape(1.0F);
  static unsigned long generated_indices[] = { 0, 1, 2, 1, 3, 2 };
  Vertex generated_vertices[4];
  static unsigned long generated_normalIndices[] = {
    0, 1,
  };
  static float generated_normals[][3] = {
    { 0.0F, 1.0F, 0.0F }, { 0.0F, 1.0F, 0.0F },
  };
  static float generated_uv[][2] = {
    { 1.0F, 1.0F }, { 0.0F, 1.0F }, { 1.0F, 0.0F }, { 0.0F, 0.0F },
  };
  shape.mass = mass;
  generated_vertices[0] = initVertex(-halfWidth, 0.0F, -halfHeight, color);
  generated_vertices[1] = initVertex(halfWidth, 0.0F, -halfHeight, color);
  generated_vertices[2] = initVertex(-halfWidth, 0.0F, halfHeight, color);
  generated_vertices[3] = initVertex(halfWidth, 0.0F, halfHeight, color);
  for(i = 0;i < 6;i++) {
    unsigned long *index = malloc(sizeof(unsigned long));
    unsigned long *uvIndex = malloc(sizeof(unsigned long));
    *index = generated_indices[i];
    *uvIndex = generated_indices[i];
    push(&shape.indices, index);
    push(&shape.uvIndices, uvIndex);
  }
  for(i = 0;i < 4;i++) {
    Vertex *vertex = malloc(sizeof(Vertex));
    float *coords = malloc(2 * sizeof(float));
    *vertex = generated_vertices[i];
    coords[0] = generated_uv[i][0];
    coords[1] = generated_uv[i][1];
    push(&shape.vertices, vertex);
    push(&shape.uv, coords);
  }
  for(i = 0;i < 2;i++) {
    float *normal = malloc(3 * sizeof(float));
    unsigned long *normalIndex = (unsigned long*)malloc(sizeof(unsigned long));
    memcpy_s(normal, 3 * sizeof(float), generated_normals[i], 3 * sizeof(float));
    *normalIndex = generated_normalIndices[i];
    push(&shape.normals, normal);
    push(&shape.normalIndices, normalIndex);
  }
  genInertiaTensorBox(100.0F * shape.mass, width, height, 0.0F, shape.inertia);
  inverse3(shape.inertia, shape.inverseInertia);
  return shape;
}

Shape initShapePlaneV(float width, float height, unsigned char color) {
  int i;
  float halfWidth = width / 2.0F;
  float halfHeight = height / 2.0F;
  Shape shape = initShape(1.0F);
  static unsigned long generated_indices[] = { 0, 2, 1, 3, 1, 2 };
  Vertex generated_vertices[4];
  static unsigned long generated_normalIndices[] = {
    0, 1,
  };
  static float generated_normals[][3] = {
    { 0.0F, 0.0F, -1.0F }, { 0.0F, 0.0F, -1.0F },
  };
  static float generated_uv[][2] = {
    { 0.0F, 1.0F }, { 0.0F, 0.0F }, { 1.0F, 1.0F }, { 1.0F, 0.0F },
  };
  generated_vertices[0] = initVertex(-halfWidth, -halfHeight, 0.0F, color);
  generated_vertices[1] = initVertex(-halfWidth, halfHeight, 0.0F, color);
  generated_vertices[2] = initVertex(halfWidth, -halfHeight, 0.0F, color);
  generated_vertices[3] = initVertex(halfWidth, halfHeight, 0.0F, color);
  for(i = 0;i < 6;i++) {
    unsigned long *index = malloc(sizeof(unsigned long));
    unsigned long *uvIndex = malloc(sizeof(unsigned long));
    *index = generated_indices[i];
    *uvIndex = generated_indices[i];
    push(&shape.indices, index);
    push(&shape.uvIndices, uvIndex);
  }
  for(i = 0;i < 4;i++) {
    Vertex *vertex = malloc(sizeof(Vertex));
    float *coords = malloc(2 * sizeof(float));
    *vertex = generated_vertices[i];
    coords[0] = generated_uv[i][0];
    coords[1] = generated_uv[i][1];
    push(&shape.vertices, vertex);
    push(&shape.uv, coords);
  }
  for(i = 0;i < 2;i++) {
    float *normal = malloc(3 * sizeof(float));
    unsigned long *normalIndex = (unsigned long*)malloc(sizeof(unsigned long));
    memcpy_s(normal, 3 * sizeof(float), generated_normals[i], 3 * sizeof(float));
    *normalIndex = generated_normalIndices[i];
    push(&shape.normals, normal);
    push(&shape.normalIndices, normalIndex);
  }
  genInertiaTensorBox(100.0F * shape.mass, width, height, 0.0F, shape.inertia);
  inverse3(shape.inertia, shape.inverseInertia);
  return shape;
}

Shape initShapePlaneInv(float width, float height, unsigned char color) {
  int i;
  float halfWidth = width / 2.0F;
  float halfHeight = height / 2.0F;
  Shape shape = initShape(1.0F);
  static unsigned long generated_indices[] = { 0, 1, 2, 1, 3, 2 };
  Vertex generated_vertices[4];
  static unsigned long generated_normalIndices[] = {
    0, 1,
  };
  static float generated_normals[][3] = {
    { 0.0F, 0.0F, 1.0F }, { 0.0F, 0.0F, 1.0F },
  };
  static float generated_uv[][2] = {
    { 0.0F, 0.0F }, { 0.0F, 1.0F }, { 1.0F, 0.0F }, { 1.0F, 1.0F },
  };
  generated_vertices[0] = initVertex(-halfWidth, -halfHeight, 0.0F, color);
  generated_vertices[1] = initVertex(-halfWidth, halfHeight, 0.0F, color);
  generated_vertices[2] = initVertex(halfWidth, -halfHeight, 0.0F, color);
  generated_vertices[3] = initVertex(halfWidth, halfHeight, 0.0F, color);
  for(i = 0;i < 6;i++) {
    unsigned long *index = malloc(sizeof(unsigned long));
    unsigned long *uvIndex = malloc(sizeof(unsigned long));
    *index = generated_indices[i];
    *uvIndex = generated_indices[i];
    push(&shape.indices, index);
    push(&shape.uvIndices, uvIndex);
  }
  for(i = 0;i < 4;i++) {
    Vertex *vertex = malloc(sizeof(Vertex));
    float *coords = malloc(2 * sizeof(float));
    *vertex = generated_vertices[i];
    coords[0] = generated_uv[i][0];
    coords[1] = generated_uv[i][1];
    push(&shape.vertices, vertex);
    push(&shape.uv, coords);
  }
  for(i = 0;i < 2;i++) {
    float *normal = malloc(3 * sizeof(float));
    unsigned long *normalIndex = (unsigned long*)malloc(sizeof(unsigned long));
    memcpy_s(normal, 3 * sizeof(float), generated_normals[i], 3 * sizeof(float));
    *normalIndex = generated_normalIndices[i];
    push(&shape.normals, normal);
    push(&shape.normalIndices, normalIndex);
  }
  genInertiaTensorBox(100.0F * shape.mass, width, height, 0.0F, shape.inertia);
  inverse3(shape.inertia, shape.inverseInertia);
  return shape;
}

Shape initShapeBox(float width, float height, float depth, unsigned char color, float mass) {
  int i;
  float halfWidth = width / 2.0F;
  float halfHeight = height / 2.0F;
  float halfDepth = depth / 2.0F;
  Shape shape = initShape(1.0F);
  static unsigned long generated_indices[] = {
    0, 1, 2, 1, 3, 2, 4, 5, 6, 5, 7, 6,
    8, 9, 10, 9, 11, 10, 12, 13, 14, 13, 15, 14,
    16, 17, 18, 17, 19, 18, 20, 21, 22, 21, 23, 22,
  };
  Vertex generated_vertices[24];
  static unsigned long generated_normalIndices[] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
  };
  static float generated_normals[][3] = {
    { 0.0F, 0.0F, 1.0F }, { 0.0F, 0.0F, 1.0F },
    { 0.0F, 0.0F, -1.0F }, { 0.0F, 0.0F, -1.0F },
    { 1.0F, 0.0F, 0.0F }, { 1.0F, 0.0F, 0.0F },
    { -1.0F, 0.0F, 0.0F }, { -1.0F, 0.0F, 0.0F },
    { 0.0F, -1.0F, 0.0F }, { 0.0F, -1.0F, 0.0F },
    { 0.0F, 1.0F, 0.0F }, { 0.0F, 1.0F, 0.0F },
  };
  static float generated_uv[][2] = {
    { 0.0F, 0.0F }, { 0.0F, 1.0F }, { 1.0F, 0.0F }, { 1.0F, 1.0F },
    { 0.0F, 0.0F }, { 1.0F, 0.0F }, { 0.0F, 1.0F }, { 1.0F, 1.0F },
    { 0.0F, 0.0F }, { 0.0F, 1.0F }, { 1.0F, 0.0F }, { 1.0F, 1.0F },
    { 0.0F, 0.0F }, { 1.0F, 0.0F }, { 0.0F, 1.0F }, { 1.0F, 1.0F },
    { 0.0F, 0.0F }, { 0.0F, 1.0F }, { 1.0F, 0.0F }, { 1.0F, 1.0F },
    { 0.0F, 0.0F }, { 1.0F, 0.0F }, { 0.0F, 1.0F }, { 1.0F, 1.0F },
  };
  shape.mass = mass;
  generated_vertices[0] = initVertex(-halfWidth, -halfHeight, halfDepth, color);
  generated_vertices[1] = initVertex(-halfWidth, halfHeight, halfDepth, color);
  generated_vertices[2] = initVertex(halfWidth, -halfHeight, halfDepth, color);
  generated_vertices[3] = initVertex(halfWidth, halfHeight, halfDepth, color);
  generated_vertices[4] = initVertex(-halfWidth, -halfHeight, -halfDepth, color);
  generated_vertices[5] = initVertex(halfWidth, -halfHeight, -halfDepth, color);
  generated_vertices[6] = initVertex(-halfWidth, halfHeight, -halfDepth, color);
  generated_vertices[7] = initVertex(halfWidth, halfHeight, -halfDepth, color);
  generated_vertices[8] = initVertex(halfWidth, -halfHeight, -halfDepth, color);
  generated_vertices[9] = initVertex(halfWidth, -halfHeight, halfDepth, color);
  generated_vertices[10] = initVertex(halfWidth, halfHeight, -halfDepth, color);
  generated_vertices[11] = initVertex(halfWidth, halfHeight, halfDepth, color);
  generated_vertices[12] = initVertex(-halfWidth, -halfHeight, -halfDepth, color);
  generated_vertices[13] = initVertex(-halfWidth, halfHeight, -halfDepth, color);
  generated_vertices[14] = initVertex(-halfWidth, -halfHeight, halfDepth, color);
  generated_vertices[15] = initVertex(-halfWidth, halfHeight, halfDepth, color);
  generated_vertices[16] = initVertex(-halfWidth, -halfHeight, -halfDepth, color);
  generated_vertices[17] = initVertex(-halfWidth, -halfHeight, halfDepth, color);
  generated_vertices[18] = initVertex(halfWidth, -halfHeight, -halfDepth, color);
  generated_vertices[19] = initVertex(halfWidth, -halfHeight, halfDepth, color);
  generated_vertices[20] = initVertex(-halfWidth, halfHeight, -halfDepth, color);
  generated_vertices[21] = initVertex(halfWidth, halfHeight, -halfDepth, color);
  generated_vertices[22] = initVertex(-halfWidth, halfHeight, halfDepth, color);
  generated_vertices[23] = initVertex(halfWidth, halfHeight, halfDepth, color);
  for(i = 0;i < 36;i++) {
    unsigned long *index = malloc(sizeof(unsigned long));
    unsigned long *uvIndex = malloc(sizeof(unsigned long));
    *index = generated_indices[i];
    *uvIndex = generated_indices[i];
    push(&shape.indices, index);
    push(&shape.uvIndices, uvIndex);
  }
  for(i = 0;i < 24;i++) {
    Vertex *vertex = malloc(sizeof(Vertex));
    float *coords = malloc(2 * sizeof(float));
    *vertex = generated_vertices[i];
    coords[0] = generated_uv[i][0];
    coords[1] = generated_uv[i][1];
    push(&shape.vertices, vertex);
    push(&shape.uv, coords);
  }
  for(i = 0;i < 12;i++) {
    float *normal = malloc(3 * sizeof(float));
    unsigned long *normalIndex = (unsigned long*)malloc(sizeof(unsigned long));
    memcpy_s(normal, 3 * sizeof(float), generated_normals[i], 3 * sizeof(float));
    *normalIndex = generated_normalIndices[i];
    push(&shape.normals, normal);
    push(&shape.normalIndices, normalIndex);
  }
  genInertiaTensorBox(100.0F * shape.mass, width, height, depth, shape.inertia);
  inverse3(shape.inertia, shape.inverseInertia);
  return shape;
}

static size_t getUntil(char *string, char *separators, size_t index, char *out, size_t out_size) {
  size_t i = 0;
  if(string[index] == '\0') {
    out[0] = '\0';
    return 0;
  }
  for(;string[index] != '\0';index++) {
    BOOL flag = FALSE;
    size_t i2;
    for(i2 = 0;separators[i2] != '\0';i2++) {
      if(string[index] == separators[i2]) {
        flag = TRUE;
        break;
      }
    }
    if(flag) continue;
    break;
  }
  for(;string[index] != '\0';index++) {
    BOOL flag = FALSE;
    size_t i2;
    for(i2 = 0;separators[i2] != '\0';i2++) {
      if(string[index] == separators[i2]) {
        flag = TRUE;
        break;
      }
    }
    if(flag) break;
    if(i < out_size - 1) {
      out[i] = string[index];
      i += 1;
    }
  }
  out[i] = '\0';
  return index + 1;
}

int initShapeFromObj(Shape *shape, char *filename, float mass) {
  FILE *file;
  char buffer[OBJ_LINE_BUFFER_SIZE];
  char temp[OBJ_WORD_BUFFER_SIZE];
  size_t line = 1;
  float aabb[3][2];
  BOOL aabbCleared = FALSE;
  *shape = initShape(mass);
  if(fopen_s(&file, filename, "r")) {
    fputs("File not found.", stderr);
    fclose(file);
    return -1;
  }
  while(fgets(buffer, OBJ_LINE_BUFFER_SIZE, file)) {
    int i;
    size_t index = 0;
    size_t old_index;
    index = getUntil(buffer, " \n", index, temp, OBJ_WORD_BUFFER_SIZE);
    if(strcmp(temp, "v") == 0) {
      Vertex *vertex = calloc(sizeof(Vertex), 1);
      if(vertex == NULL) {
        fputs("readObj: Memory allocation failed.", stderr);
        fclose(file);
        return -2;
      }
      for(i = 0;i < 4;i++) {
        old_index = index;
        index = getUntil(buffer, " \n", index, temp, OBJ_WORD_BUFFER_SIZE);
        if(temp[0] == '\0') {
          if(i == 3) {
            vertex->components[3] = 1.0F;
          } else {
            fprintf(stderr, "readObj: Vertex component %d does not found. (%zu, %zu)", i, line, old_index);
            fclose(file);
            return -3;
          }
        } else {
          vertex->components[i] = (float)atof(temp);
        }
      }
      if(aabbCleared) {
        aabb[0][0] = min(aabb[0][0], vertex->components[0]);
        aabb[0][1] = max(aabb[0][1], vertex->components[0]);
        aabb[1][0] = min(aabb[1][0], vertex->components[1]);
        aabb[1][1] = max(aabb[1][1], vertex->components[1]);
        aabb[2][0] = min(aabb[2][0], vertex->components[2]);
        aabb[2][1] = max(aabb[2][1], vertex->components[2]);
      } else {
        aabb[0][0] = vertex->components[0];
        aabb[0][1] = vertex->components[0];
        aabb[1][0] = vertex->components[1];
        aabb[1][1] = vertex->components[1];
        aabb[2][0] = vertex->components[2];
        aabb[2][1] = vertex->components[2];
        aabbCleared = TRUE;
      }
      push(&shape->vertices, vertex);
    } else if(strcmp(temp, "vt") == 0) {
      float *coords = malloc(2 * sizeof(float));
      if(coords == NULL) {
        fputs("readObj: Memory allocation failed.", stderr);
        fclose(file);
        return -4;
      }
      for(i = 0;i < 2;i++) {
        old_index = index;
        index = getUntil(buffer, " \n", index, temp, OBJ_WORD_BUFFER_SIZE);
        if(temp[0] == '\0') {
          if(i != 0) {
            coords[1] = 0.0F;
          } else {
            fprintf(stderr, "readObj: Terxture cordinates' component %d does not found. (%zu, %zu)", i, line, old_index);
            fclose(file);
            return -5;
          }
        } else {
          if(i == 1) {
            coords[i] = 1.0F - (float)atof(temp);
          } else {
            coords[i] = (float)atof(temp);
          }
        }
      }
      push(&shape->uv, coords);
    } else if(strcmp(temp, "vn") == 0) {
      float normalTemp[3];
      float *normal = malloc(3 * sizeof(float));
      if(normal == NULL) {
        fputs("readObj: Memory allocation failed.", stderr);
        fclose(file);
        return -9;
      }
      for(i = 0;i < 3;i++) {
        old_index = index;
        index = getUntil(buffer, " \n", index, temp, OBJ_WORD_BUFFER_SIZE);
        if(temp[0] == '\0') {
          fprintf(stderr, "readObj: Vertex component %d does not found. (%zu, %zu)", i, line, old_index);
          fclose(file);
          return -10;
        } else {
          normalTemp[i] = (float)atof(temp);
        }
      }
      normalize3(normalTemp, normal);
      push(&shape->normals, normal);
    } else if(strcmp(temp, "f") == 0) {
      unsigned long faceIndices[3];
      unsigned long faceUVIndices[3];
      unsigned long *normalIndex = malloc(sizeof(unsigned long));
      if(normalIndex == NULL) {
        fputs("readObj: Memory allocation failed.", stderr);
        fclose(file);
        return -11;
      }
      for(i = 0;i < 3;i++) {
        old_index = index;
        index = getUntil(buffer, " \n", index, temp, OBJ_WORD_BUFFER_SIZE);
        if(temp[0] == '\0') {
          fprintf(stderr, "readObj: Vertex component %d does not found. (%zu, %zu)", i, line, old_index);
          fclose(file);
          return -6;
        } else {
          size_t index2 = 0;
          char temp2[OBJ_WORD_BUFFER_SIZE];
          index2 = getUntil(temp, "/", index2, temp2, OBJ_WORD_BUFFER_SIZE);
          faceIndices[i] = atoi(temp2);
          if(faceIndices[i] < 0) {
            faceIndices[i] += shape->vertices.length;
          } else {
            faceIndices[i] -= 1;
          }
          index2 = getUntil(temp, "/", index2, temp2, OBJ_WORD_BUFFER_SIZE);
          if(temp2[0] == '\0') {
            faceUVIndices[i] = 0;
          } else {
            faceUVIndices[i] = atoi(temp2);
            if(faceUVIndices[i] < 0) {
              faceUVIndices[i] += shape->uv.length;
            } else {
              faceUVIndices[i] -= 1;
            }
          }
          if(i == 0) {
            getUntil(temp, "", index2, temp2, OBJ_WORD_BUFFER_SIZE);
            if(temp2[0] == '\0') {
              *normalIndex = 0;
            } else {
              *normalIndex = atoi(temp2);
              if(*normalIndex < 0) {
                *normalIndex += shape->normals.length;
              } else {
                *normalIndex -= 1;
              }
            }
            push(&shape->normalIndices, normalIndex);
          }
        }
      }
      for(i = 2;i >= 0;i--) {
        unsigned long *faceIndex = malloc(sizeof(unsigned long));
        unsigned long *faceUVIndex = malloc(sizeof(unsigned long));
        if(faceIndex == NULL || faceUVIndex == NULL) {
          fputs("readObj: Memory allocation failed.", stderr);
          fclose(file);
          return -7;
        }
        *faceIndex = faceIndices[i];
        *faceUVIndex = faceUVIndices[i];
        push(&shape->indices, faceIndex);
        push(&shape->uvIndices, faceUVIndex);
      }
    } else if(temp[0] != '\0' && temp[0] != '#' && strcmp(temp, "vn") != 0 &&
              strcmp(temp, "vp") != 0 && strcmp(temp, "l") != 0 && strcmp(temp, "mtllib") != 0 &&
              strcmp(temp, "o") != 0 && strcmp(temp, "usemtl") != 0 && strcmp(temp, "s") != 0) {
      fprintf(stderr, "readObj: Unexpected word '%s' (%zu)", temp, line);
      fclose(file);
      return -8;
    }
    line += 1;
  }
  genInertiaTensorBox(100.0F * shape->mass, fabsf(aabb[0][1] - aabb[0][0]), fabsf(aabb[1][1] - aabb[1][0]), fabsf(aabb[2][1] - aabb[2][0]), shape->inertia);
  inverse3(shape->inertia, shape->inverseInertia);
  fclose(file);
  return 0;
}

float (*getShapeAABB(Shape shape, float transformation[4][4], float out[3][2]))[2] {
  Vertex *vertex;
  int first = TRUE;
  iterf(&shape.vertices, &vertex) {
    float transformed[4];
    mulMat4Vec4(transformation, vertex->components, transformed);
    if(first) {
      out[0][0] = transformed[0];
      out[0][1] = transformed[0];
      out[1][0] = transformed[1];
      out[1][1] = transformed[1];
      out[2][0] = transformed[2];
      out[2][1] = transformed[2];
      first = FALSE;
    } else {
      out[0][0] = min(out[0][0], transformed[0]);
      out[0][1] = max(out[0][1], transformed[0]);
      out[1][0] = min(out[1][0], transformed[1]);
      out[1][1] = max(out[1][1], transformed[1]);
      out[2][0] = min(out[2][0], transformed[2]);
      out[2][1] = max(out[2][1], transformed[2]);
    }
  }
  return out;
}

void discardShape(Shape shape) {
  freeVector(&shape.indices);
  freeVector(&shape.vertices);
  freeVector(&shape.uv);
  freeVector(&shape.uvIndices);
}
