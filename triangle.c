#include<Windows.h>
#include<math.h>

#include "./include/borland.h"
#include "./include/triangle.h"
#include "./include/matrix.h"

float* point3dTo2d(const float point[3], const float vec[2][3], float out[2]) {
  out[0] = dot3(vec[0], point);
  out[1] = dot3(vec[1], point);
  return out;
}

void triangle3dTo2d(const float triangle[3][3], float triangleOut[3][2], float vecOut[2][3]) {
  float tempVec3[2][3];
  float normal[3];
  normalize3(subVec3(triangle[0], triangle[1], tempVec3[0]), vecOut[0]);
  normalize3(cross(subVec3(triangle[2], triangle[1], tempVec3[0]), vecOut[0], tempVec3[1]), normal);
  normalize3(cross(vecOut[0], normal, tempVec3[0]), vecOut[1]);
  point3dTo2d(triangle[0], vecOut, triangleOut[0]);
  point3dTo2d(triangle[1], vecOut, triangleOut[1]);
  point3dTo2d(triangle[2], vecOut, triangleOut[2]);
}

float* cartesianToBarycentric(const float triangle[3][2], const float point[2], float barycentric[3]) {
  float det;
  det = (triangle[1][1] - triangle[2][1]) * (triangle[0][0] - triangle[2][0]) + (triangle[2][0] - triangle[1][0]) * (triangle[0][1] - triangle[2][1]);
  barycentric[0] = ((triangle[1][1] - triangle[2][1]) * (point[0] - triangle[2][0]) + (triangle[2][0] - triangle[1][0]) * (point[1] - triangle[2][1])) / det;
  barycentric[1] = ((triangle[2][1] - triangle[0][1]) * (point[0] - triangle[2][0]) + (triangle[0][0] - triangle[2][0]) * (point[1] - triangle[2][1])) / det;
  barycentric[2] = 1.0F - barycentric[0] - barycentric[1];
  return barycentric;
}

float* cartesian3dToBarycentric(const float triangle[3][3], const float point[3], float barycentric[3]) {
    float triangle2d[3][2], vec2d[2][3];
    float point2d[2];
    triangle3dTo2d(triangle, triangle2d, vec2d);
    point3dTo2d(point, vec2d, point2d);
    cartesianToBarycentric(triangle2d, point2d, barycentric);
    return barycentric;
}

static int calcPlaneEquation(const float triangle[3][3], const float target[3][3], float n[3], float *d, float dv[3]) {
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

static float calcLineParameters(const float triangle[3][3], const float d[3], const float dv[3], float t[2], float vertices[2][3]) {
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
  return (dv[indexB] < 0) ? dv[indexB] : min(dv[indexA], dv[indexC]);
}

int testCollisionTriangleTriangle(const float a[3][3], const float b[3][3], float contacts[2][3], float depths[2]) {
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
  if(dv2[0] == 0.0F && dv2[1] == 0.0F && dv2[2] == 0.0F) return FALSE;
  cross(n1, n2, tempVec3[0]);
  normalize3(tempVec3[0], d);
  depths[1] = calcLineParameters(a, d, dv2, t1, v1);
  depths[0] = calcLineParameters(b, d, dv1, t2, v2);
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
    memcpy_s(contacts[0], SIZE_VEC3, (t1MinMax[0] > t2MinMax[0]) ? v1MinMax[0] : v2MinMax[0], SIZE_VEC3);
    memcpy_s(contacts[1], SIZE_VEC3, (t1MinMax[1] < t2MinMax[1]) ? v1MinMax[1] : v2MinMax[1], SIZE_VEC3);
    return TRUE;
  }
  return FALSE;
}
