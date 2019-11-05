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

#define OBJ_LINE_BUFFER_SIZE 128
#define OBJ_WORD_BUFFER_SIZE 32

Node initNode(const char *id, Image image) {
  Node node;
  memset(&node, 0, sizeof(Node));
  memcpy_s(node.id, sizeof(node.id), id, min(sizeof(node.id), strlen(id)));
	node.scale[0] = 1.0;
	node.scale[1] = 1.0;
  node.scale[2] = 1.0;
	node.texture = image;
  node.children = initVector();
  node.isVisible = TRUE;
  return node;
}

Node initNodeUI(const char *id, Image image, unsigned char color) {
  Node node;
  node = initNode(id, image);
  node.shape = initShapePlaneInv(1.0F, 1.0F, color);
  node.isInterface = TRUE;
  return node;
}

Node initNodeText(const char *id, float px, float py, unsigned int sx, unsigned int sy, int (*behaviour)(Node*)) {
  Node node;
  unsigned int size[2];
  getScreenSize(size);
  node = initNodeUI(id, initImage(sx, sy, BLACK, BLACK), NULL_COLOR);
  node.position[0] = px * 100.0F / size[0] + (sign(px) == -1 ? 100.0F : 0.0F);
  node.position[1] = py * 100.0F / size[1] + (sign(py) == -1 ? 100.0F : 0.0F);
	node.scale[0] = sx * 100.0F / size[0];
	node.scale[1] = sy * 100.0F / size[1];
  node.behaviour = behaviour;
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

void discardNode(Node node) {
  clearVector(&node.children);
}

void drawNode(Node *node) {
  Node *child;
	pushTransformation();
  if(node->isInterface) {
    translateTransformation((node->position[0] + node->scale[0] / 2.0F) / 50.0F - 1.0F, (node->position[1] + node->scale[1] / 2.0F) / 50.0F - 1.0F, 0.0F);
  } else {
    translateTransformation(node->position[0], node->position[1], node->position[2]);
  }
  rotateTransformation(node->angle[0], node->angle[1], node->angle[2]);
  pushTransformation();
  if(node->isInterface) {
    clearCameraMat4();
    scaleTransformation(node->scale[0] / 50.0F, node->scale[1] / 50.0F, 1.0F);
  } else {
    scaleTransformation(node->scale[0], node->scale[1], node->scale[2]);
  }
  getTransformation(node->lastTransformation);
  if(node->isVisible) {
    clearAABB();
    fillPolygons(node->shape.vertices, node->shape.indices, node->texture, node->shape.uv, node->shape.uvIndices);
    getAABB(node->aabb);
  } else {
    getShapeAABB(node->shape, node->lastTransformation, node->aabb);
  }
  popTransformation();
  resetIteration(&node->children);
  child = previousData(&node->children);
  while(child) {
    drawNode(child);
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

float (*getWorldTransfomration(Node node, float out[4][4]))[4] {
  Node *current = &node;
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

int calcPlaneEquation(const float triangle[3][3], const float target[3][3], float n[3], float *d, float dv[3]) {
  float temp[3][3];
  cross(subVec3(triangle[1], triangle[0], temp[0]), subVec3(triangle[2], triangle[0], temp[1]), temp[2]);
  normalize3(temp[2], n);
  *d = - dot3(n, triangle[0]);
  dv[0] = dot3(n, target[0]) + *d;
  dv[1] = dot3(n, target[1]) + *d;
  dv[2] = dot3(n, target[2]) + *d;
  if(dv[0] != 0.0F && dv[1] != 0.0F && dv[2] != 0.0F) {
    int signDv1 = sign(dv[1]);
    if(sign(dv[0]) == signDv1 && signDv1 == sign(dv[2])) {
      return -1;
    }
  }
  return 0;
}

static void calcLineParameters(const float triangle[3][3], const float d[3], const float dv[3], float t[2], float vertices[2][3]) {
  float tempVec3[2][3];
  float pv[3];
  int signA, signB, signC;
  int indexA, indexB, indexC;
  float ratio[2];
  signA = sign(dv[0]);
  signB = sign(dv[1]);
  signC = sign(dv[2]);
  if(signA == signB) {
    indexA = 0;
    indexB = 2;
    indexC = 1;
  } else if(signB == signC) {
    indexA = 1;
    indexB = 0;
    indexC = 2;
  } else {
    indexA = 0;
    indexB = 1;
    indexC = 2;
  }
  pv[0] = dot3(d, triangle[0]);
  pv[1] = dot3(d, triangle[1]);
  pv[2] = dot3(d, triangle[2]);
  ratio[0] = dv[indexA] / (dv[indexA] - dv[indexB]);
  ratio[1] = dv[indexC] / (dv[indexC] - dv[indexB]);
  t[0] = pv[indexA] + (pv[indexB] - pv[indexA]) * ratio[0];
  t[1] = pv[indexC] + (pv[indexB] - pv[indexC]) * ratio[1];
  addVec3(triangle[indexA], mulVec3ByScalar(subVec3(triangle[indexB], triangle[indexA], tempVec3[0]), ratio[0], tempVec3[1]), vertices[0]);
  addVec3(triangle[indexC], mulVec3ByScalar(subVec3(triangle[indexB], triangle[indexC], tempVec3[0]), ratio[1], tempVec3[1]), vertices[1]);
}

static int testCollisionTriangleTriangle(const float a[3][3], const float b[3][3], Vector *points) {
  // using Moller's algorithm: A Fast Triangle-Triangle Intersection Test
  float tempVec3[2][3];
  float n1[3], n2[3];
  float d1, d2;
  float dv1[3], dv2[3];
  float d[3];
  float t1[2], t2[2];
  float v1[2][3], v2[2][3];
  float *v1MinMax[2], *v2MinMax[2];
  float t1MinMax[2], t2MinMax[2];
  if(calcPlaneEquation(b, a, n2, &d2, dv2) || calcPlaneEquation(a, b, n1, &d1, dv1)) return FALSE;
  cross(n1, n2, tempVec3[0]);
  normalize3(tempVec3[0], d);
  calcLineParameters(a, d, dv2, t1, v1);
  calcLineParameters(b, d, dv1, t2, v2);
  if(t1[0] > t1[1]) {
    t1MinMax[0] = t1[1];
    t1MinMax[1] = t1[0];
    v1MinMax[0] = v1[1];
    v1MinMax[1] = v1[0];
  } else {
    t1MinMax[0] = t1[0];
    t1MinMax[1] = t1[1];
    v1MinMax[0] = v1[0];
    v1MinMax[1] = v1[1];
  }
  if(t2[0] > t2[1]) {
    t2MinMax[0] = t2[1];
    t2MinMax[1] = t2[0];
    v2MinMax[0] = v2[1];
    v2MinMax[1] = v2[0];
  } else {
    t2MinMax[0] = t2[0];
    t2MinMax[1] = t2[1];
    v2MinMax[0] = v2[0];
    v2MinMax[1] = v2[1];
  }
  if(!(t1MinMax[1] < t2MinMax[0] || t2MinMax[1] < t1MinMax[0])) {
    float *contacts[2];
    contacts[0] = (t1MinMax[0] > t2MinMax[0]) ? v1MinMax[0] : v2MinMax[0];
    contacts[1] = (t1MinMax[1] < t2MinMax[1]) ? v1MinMax[1] : v2MinMax[1];
    pushAllocUntilNull(points, SIZE_VEC3, contacts[0], contacts[1], NULL);
    return 2;
  }
  return 0;
}

float (*mulMat4ByTriangle(float mat[4][4], float triangle[3][3], float out[3][3]))[3] {
  int i;
  for(i = 0;i < 3;i++) {
    float point[4];
    float transformed[4];
    COPY_ARY(point, triangle[i]);
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

int testCollisionPolygonPolygon(Node a, Node b, Vector *normals, Vector *points) {
  int i;
  unsigned long indexA = 0;
  Vector polygonsA = initVector();
  Vector polygonsB = initVector();
  float (*polygonA)[3], (*polygonB)[3];
  int collided = FALSE;
  getPolygons(a, &polygonsA);
  getPolygons(b, &polygonsB);
  polygonA = nextData(&polygonsA);
  while(polygonA) {
    unsigned long indexB = 0;
    float polygonAWorld[3][3];
    mulMat4ByTriangle(a.lastTransformation, polygonA, polygonAWorld);
    resetIteration(&polygonsB);
    polygonB = nextData(&polygonsB);
    while(polygonB) {
      float polygonBWorld[3][3];
      int nofNormals;
      mulMat4ByTriangle(b.lastTransformation, polygonB, polygonBWorld);
      nofNormals = testCollisionTriangleTriangle(polygonAWorld, polygonBWorld, points);
      if(nofNormals) {
        float temp[2][3];
        float tempMat3[2][3][3];
        float (*normal)[3] = malloc(6 * sizeof(float));
        unsigned long *normalIndex;
        normalIndex = (unsigned long*)dataAt(&a.collisionShape.normalIndices, indexA);
        memcpy_s(temp, 3 * sizeof(float), dataAt(&a.collisionShape.normals, *normalIndex), 3 * sizeof(float));
        transposeMat3(inverse3(convMat4toMat3(a.lastTransformation, tempMat3[0]), tempMat3[1]), tempMat3[0]);
        mulMat3Vec3(tempMat3[0], temp[0], temp[1]);
        normalize3(temp[1], normal[0]);
        normalIndex = (unsigned long*)dataAt(&b.collisionShape.normalIndices, indexB);
        memcpy_s(temp, 3 * sizeof(float), dataAt(&b.collisionShape.normals, *normalIndex), 3 * sizeof(float));
        transposeMat3(inverse3(convMat4toMat3(b.lastTransformation, tempMat3[0]), tempMat3[1]), tempMat3[0]);
        mulMat3Vec3(tempMat3[0], temp[0], temp[1]);
        normalize3(temp[1], normal[1]);
        for(i = 0;i < nofNormals;i++) push(normals, normal);
        collided = TRUE;
      }
      polygonB = nextData(&polygonsB);
      indexB += 1;
    }
    polygonA = nextData(&polygonsA);
    indexA += 1;
  }
  freeVector(&polygonsA);
  freeVector(&polygonsB);
  return collided;
}

void addIntervalEventNode(Node *node, unsigned int milliseconds, void (*callback)(Node*)) {
  IntervalEventNode *interval = malloc(sizeof(IntervalEventNode));
  interval->begin = clock();
  interval->interval = milliseconds * CLOCKS_PER_SEC / 1000;
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
  shape.restitution = 0.0F;
  shape.staticFriction = 0.5F;
  shape.dynamicFriction = 0.3F;
  shape.rollingFriction = 0.5F;
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
