#ifndef TRIANGLE_H
#define TRIANGLE_H

float* point3dTo2d(const float point[3], const float vec[2][3], float out[2]);
void triangle3dTo2d(const float triangle[3][3], float triangleOut[3][2], float vecOut[2][3]);
float* cartesianToBarycentric(const float triangle[3][2], const float point[2], float barycentric[3]);
float* cartesian3dToBarycentric(const float triangle[3][3], const float point[3], float barycentric[3]);
int testCollisionTriangleTriangle(const float a[3][3], const float b[3][3], float contacts[2][3], float depths[2]);

#endif
