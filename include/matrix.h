#ifndef MATRIX_H
#define MATRIX_H

#define PI  3.14159265359F

#define sign(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))
#define COPY_ARY(dest, src) memcpy_s(dest, sizeof(dest), src, sizeof(src))

#define SIZE_VEC2 (2 * sizeof(float))
#define SIZE_VEC3 (3 * sizeof(float))
#define SIZE_MAT4 (16 * sizeof(float))

#define X_MASK 1
#define Y_MASK 2
#define XY_MASK 3
#define Z_MASK 4
#define XZ_MASK 5
#define YZ_MASK 6
#define XYZ_MASK 7

float *convVec3toVec4(const float in[3], float out[4]);
float *convVec4toVec3(const float in[4], float out[3]);
float *extractComponents3(const float in[3], int mask, float out[3]);

float (*convMat4toMat3(float in[4][4], float out[3][3]))[3];

float* initVec3(float vec[3], int mask);
float* setVec3(float vec[3], float val, int mask);
int equalVec3(float a[3], float b[3]);
float cosVec3(float a[3], float b[3]);
float *clearVec3(float in[3]);

float dot2(const float a[2], const float b[2]);
float dot3(const float a[3], const float b[3]);
float dot4(const float a[4], const float b[4]);
float* cross(const float a[3], const float b[3], float out[3]);
float areaOfTriangle(float triangle[3][3]);
float length2(const float vector[2]);
float length3(const float vector[3]);
float distance2(const float a[2], const float b[2]);
float distance3(const float a[3], const float b[3]);
float distancePoint2(const float point[2], const float vector[2]);
float* addVec2(const float a[2], const float b[2], float out[2]);
float* addVec3(const float a[3], const float b[3], float out[3]);
float* subVec2(const float a[2], const float b[2], float out[2]);
float* subVec3(const float a[3], const float b[3], float out[3]);
float* mulVec2ByScalar(const float vector[2], float scalar, float out[2]);
float* divVec2ByScalar(const float vector[2], float scalar, float out[2]);
float* mulVec3ByScalar(const float vector[3], float scalar, float out[3]);
float* divVec3ByScalar(const float vector[3], float scalar, float out[3]);
float* mulVec4ByScalar(const float vector[4], float scalar, float out[4]);
float* divVec4ByScalar(const float vector[4], float scalar, float out[4]);
float* normalize2(const float vector[2], float out[2]);
float* normalize3(const float vector[3], float out[3]);
float* direction2(const float a[2], const float b[2], float out[2]);
float* direction3(const float a[3], const float b[3], float out[3]);
float angleVec2(const float vector[2]);

float (*addMat3(const float a[3][3], const float b[3][3], float out[3][3]))[3];
float	(*mulMat3(const float a[3][3], const float b[3][3], float out[3][3]))[3];
float	(*mulMat4(const float a[4][4], const float b[4][4], float out[4][4]))[4];
float	(*mulMat3ByScalar(const float in[3][3], float scalar, float out[3][3]))[3];
float	(*divMat3ByScalar(const float in[3][3], float scalar, float out[3][3]))[3];
float	(*divMat4ByScalar(const float in[4][4], float scalar, float out[4][4]))[4];
float* mulMat3Vec3(const float mat[3][3], const float vec[3], float out[3]);
float* mulMat4Vec4(const float mat[4][4], const float vec[4], float out[4]);
float* mulMat4Vec4Proj(const float mat[4][4], const float vec[4], float out[4]);
float (*transposeMat3(const float mat[3][3], float out[3][3]))[3];
float (*transposeMat4(const float mat[4][4], float out[4][4]))[4];

float (*cofactor3(const float in[3][3], int row, int col, float out[2][2]))[2];
float (*cofactor4(const float in[4][4], int row, int col, float out[3][3]))[3];
float (*cofactorsMat3(const float in[3][3], float out[3][3]))[3];
float (*cofactorsMat4(const float in[4][4], float out[4][4]))[4];
float (*adjugate3(float in[3][3], float out[3][3]))[3];
float (*adjugate4(float in[4][4], float out[4][4]))[4];
float det2(float in[2][2]);
float det3(float in[3][3]);
float det4(float in[4][4]);
float (*inverse3(float in[3][3], float out[3][3]))[3];
float (*inverse4(float in[4][4], float out[4][4]))[4];
float (*orthogonalize3(float in[3][3], float out[3][3]))[3];

float *getTriangleCM3(float triangle[3][3], float out[3]);

float (*genSkewMat3(float in[3], float out[3][3]))[3];
float (*genTranslationMat3(float dx, float dy, float mat[3][3]))[3];
float (*genTranslationMat4(float dx, float dy, float dz, float mat[4][4]))[4];
float (*genIdentityMat3(float mat[3][3]))[3];
float (*genIdentityMat4(float mat[4][4]))[4];
float (*genScaleMat3(float sx, float sy, float mat[3][3]))[3];
float (*genScaleMat4(float sx, float sy, float sz, float mat[4][4]))[4];
float (*genRotationMat3(float rotation, float mat[3][3]))[3];
float (*genRotationXMat4(float rotation, float mat[4][4]))[4];
float (*genRotationYMat4(float rotation, float mat[4][4]))[4];
float (*genRotationZMat4(float rotation, float mat[4][4]))[4];
float (*genRotationMat4(float rx, float ry, float rz, float mat[4][4]))[4];

float *getAngleFromMat3(float in[3][3], float out[3]);

float (*genLookAtMat4(float position[3], float target[3], float worldUp[3], float mat[4][4]))[4];
float (*genPerspectiveMat4(float fovY, float zNear, float zFar, float aspect, float mat[4][4]))[4];

float (*genInertiaTensorBox(float mass, float width, float height, float depth, float out[3][3]))[3];

void printVec3(float vec[3]);
void printVec4(float vec[4]);
void printMat3(float mat[3][3]);
void printMat4(float mat[4][4]);

#endif
