#include<stdio.h>
#include<float.h>
#include<math.h>
#include<Windows.h>

#include "./include/matrix.h"
#include "./include/borland.h"

float *convVec3toVec4(const float in[3], float out[4]) {
	memcpy_s(out, 4 * sizeof(float), in, 3 * sizeof(float));
	out[3] = 1.0F;
	return out;
}

float *convVec4toVec3(const float in[4], float out[3]) {
	memcpy_s(out, 3 * sizeof(float), in, 3 * sizeof(float));
	return out;
}

float *extractComponents3(const float in[3], int mask, float out[3]) {
	out[0] = (mask & X_MASK) ? in[0] : 0.0F;
	out[1] = (mask & Y_MASK) ? in[1] : 0.0F;
	out[2] = (mask & Z_MASK) ? in[2] : 0.0F;
	return out;
}

float (*convMat4toMat3(float in[4][4], float out[3][3]))[3] {
	memcpy_s(out[0], 3 * sizeof(float), in[0], 3 * sizeof(float));
	memcpy_s(out[1], 3 * sizeof(float), in[1], 3 * sizeof(float));
	memcpy_s(out[2], 3 * sizeof(float), in[2], 3 * sizeof(float));
	return out;
}

float* initVec3(float vec[3], int mask) {
	vec[0] = (mask & X_MASK) ? 1.0F : 0.0F;
	vec[1] = (mask & Y_MASK) ? 1.0F : 0.0F;
	vec[2] = (mask & Z_MASK) ? 1.0F : 0.0F;
	return vec;
}

int equalVec3(float a[3], float b[3]) {
	if(a[0] == b[0] && a[1] == b[1] && a[2] == b[2]) return TRUE;
	return FALSE;
}

float cosVec3(float a[3], float b[3]) {
	return dot3(a, b) / (length3(a) * length3(b));
}

float *clearVec3(float in[3]) {
	memset(in, 0, 3 * sizeof(float));
	return in;
};

float dot2(const float a[2], const float b[2]) {
	return a[0] * b[0] + a[1] * b[1];
}

float dot3(const float a[3], const float b[3]) {
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

float dot4(const float a[4], const float b[4]) {
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
}

float* cross(const float a[3], const float b[3], float out[3]) {
	out[0] = a[1] * b[2] - a[2] * b[1];
	out[1] = a[2] * b[0] - a[0] * b[2];
	out[2] = a[0] * b[1] - a[1] * b[0];
	return out;
}

float areaOfTriangle(float triangle[3][3]) {
  float vecA[3], vecB[3];
  float temp[3];
  subVec3(triangle[1], triangle[0], vecA);
  subVec3(triangle[2], triangle[0], vecB);
  cross(vecA, vecB, temp);
  return length3(temp) / 2.0F;
}

float length2(const float vector[2]) {
	return sqrtf(vector[0] * vector[0] + vector[1] * vector[1]);
}

float length3(const float vector[3]) {
	return sqrtf(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
}

float distance2(const float a[2], const float b[2]) {
	float position[2];
	position[0] = b[0] - a[0];
	position[1] = b[1] - a[1];
	return length2(position);
}

float distance3(const float a[3], const float b[3]) {
	float position[3];
	position[0] = b[0] - a[0];
	position[1] = b[1] - a[1];
	position[2] = b[2] - a[2];
	return length3(position);
}

float distancePoint2(const float point[2], const float vector[2]) {
	float vectorLen = length2(vector);
	float projectedPointLen;
	float projectedPoint[2];
	float difference[2];
	if(vectorLen == 0.0F) return length2(point);
	projectedPointLen = dot2(point, vector) / vectorLen;
	normalize2(vector, projectedPoint);
	mulVec2ByScalar(projectedPoint, projectedPointLen, projectedPoint);
	subVec2(point, projectedPoint, difference);
	return length2(difference);
}

float* addVec2(const float a[2], const float b[2], float out[2]) {
	out[0] = a[0] + b[0];
	out[1] = a[1] + b[1];
	return out;
}

float* addVec3(const float a[3], const float b[3], float out[3]) {
	out[0] = a[0] + b[0];
	out[1] = a[1] + b[1];
	out[2] = a[2] + b[2];
	return out;
}

float* subVec2(const float a[2], const float b[2], float out[2]) {
	out[0] = a[0] - b[0];
	out[1] = a[1] - b[1];
	return out;
}

float* subVec3(const float a[3], const float b[3], float out[3]) {
	out[0] = a[0] - b[0];
	out[1] = a[1] - b[1];
	out[2] = a[2] - b[2];
	return out;
}

float* mulVec2ByScalar(const float vector[2], float scalar, float out[2]) {
	out[0] = scalar * vector[0];
	out[1] = scalar * vector[1];
	return out;
}

float* divVec2ByScalar(const float vector[2], float scalar, float out[2]) {
	out[0] = vector[0] / scalar;
	out[1] = vector[1] / scalar;
	return out;
}

float* mulVec3ByScalar(const float vector[3], float scalar, float out[3]) {
	out[0] = scalar * vector[0];
	out[1] = scalar * vector[1];
	out[2] = scalar * vector[2];
	return out;
}

float* divVec3ByScalar(const float vector[3], float scalar, float out[3]) {
	out[0] = vector[0] / scalar;
	out[1] = vector[1] / scalar;
	out[2] = vector[2] / scalar;
	return out;
}

float* mulVec4ByScalar(const float vector[4], float scalar, float out[4]) {
	out[0] = scalar * vector[0];
	out[1] = scalar * vector[1];
	out[2] = scalar * vector[2];
	out[3] = scalar * vector[3];
	return out;
}

float* divVec4ByScalar(const float vector[4], float scalar, float out[4]) {
	out[0] = vector[0] / scalar;
	out[1] = vector[1] / scalar;
	out[2] = vector[2] / scalar;
	out[3] = vector[3] / scalar;
	return out;
}

float* normalize2(const float vector[2], float out[2]) {
	float length = length2(vector);
	if(length == 0.0F) {
		out[0] = 0.0F;
		out[1] = 0.0F;
	} else {
		out[0] = vector[0] / length;
		out[1] = vector[1] / length;
	}
	return out;
}

float* normalize3(const float vector[3], float out[3]) {
	float length = length3(vector);
	if(length == 0.0F) {
		out[0] = 0.0F;
		out[1] = 0.0F;
		out[2] = 0.0F;
	} else {
		out[0] = vector[0] / length;
		out[1] = vector[1] / length;
		out[2] = vector[2] / length;
	}
	return out;
}

float* direction2(const float a[2], const float b[2], float out[2]) {
	float temp[2];
	subVec2(a, b, temp);
	normalize2(temp, out);
	return out;
}

float* direction3(const float a[3], const float b[3], float out[3]) {
	float temp[3];
	subVec3(a, b, temp);
	normalize3(temp, out);
	return out;
}

float angleVec2(const float vector[2]) {
	float result;
	if(vector[0] == 0.0F) {
		result = (vector[1] == 0.0F || vector[1] < 0.0F) ? 3.0F * PI / 2.0F : PI / 2.0F;
	} else {
		if(vector[1] == 0.0F) {
			result = (vector[0] > 0.0F) ? 0.0F : PI;
		} else {
			result = atanf(vector[1] / vector[0]);
			if(vector[0] < 0.0F) {
				result += PI;
			} else {
				if(result < 0.0F) result += 2.0F * PI;
			}
		}
	}
	return result;
}

float (*addMat3(const float a[3][3], const float b[3][3], float out[3][3]))[3] {
	int row, col;
	for(row = 0;row < 3;row++) {
		for(col = 0;col < 3;col++) out[row][col] = a[row][col] + b[row][col];
	}
	return out;
}

float	(*mulMat3(const float a[3][3], const float b[3][3], float out[3][3]))[3] {
	float b0[3], b1[3], b2[3];
	b0[0] = b[0][0];
	b0[1] = b[1][0];
	b0[2] = b[2][0];
	b1[0] = b[0][1];
	b1[1] = b[1][1];
	b1[2] = b[2][1];
	b2[0] = b[0][2];
	b2[1] = b[1][2];
	b2[2] = b[2][2];
	out[0][0] = dot3(a[0], b0);
	out[0][1] = dot3(a[0], b1);
	out[0][2] = dot3(a[0], b2);
	out[1][0] = dot3(a[1], b0);
	out[1][1] = dot3(a[1], b1);
	out[1][2] = dot3(a[1], b2);
	out[2][0] = dot3(a[2], b0);
	out[2][1] = dot3(a[2], b1);
	out[2][2] = dot3(a[2], b2);
	return out;
}

float	(*mulMat3ByScalar(const float in[3][3], float scalar, float out[3][3]))[3] {
	out[0][0] = scalar * in[0][0];
	out[0][1] = scalar * in[0][1];
	out[0][2] = scalar * in[0][2];
	out[1][0] = scalar * in[1][0];
	out[1][1] = scalar * in[1][1];
	out[1][2] = scalar * in[1][2];
	out[2][0] = scalar * in[2][0];
	out[2][1] = scalar * in[2][1];
	out[2][2] = scalar * in[2][2];
	return out;
}

float	(*divMat3ByScalar(const float in[3][3], float scalar, float out[3][3]))[3] {
	out[0][0] = in[0][0] / scalar;
	out[0][1] = in[0][1] / scalar;
	out[0][2] = in[0][2] / scalar;
	out[1][0] = in[1][0] / scalar;
	out[1][1] = in[1][1] / scalar;
	out[1][2] = in[1][2] / scalar;
	out[2][0] = in[2][0] / scalar;
	out[2][1] = in[2][1] / scalar;
	out[2][2] = in[2][2] / scalar;
	return out;
}

float	(*divMat4ByScalar(const float in[4][4], float scalar, float out[4][4]))[4] {
	int r, c;
	for(r = 0;r < 4;r++) {
		for(c = 0;c < 4;c++) out[r][c] = in[r][c] / scalar;
	}
	return out;
}

float	(*mulMat4(const float a[4][4], const float b[4][4], float out[4][4]))[4] {
	float b0[4];
	float b1[4];
	float b2[4];
	float b3[4];
	b0[0] = b[0][0];
	b0[1] = b[1][0];
	b0[2] = b[2][0];
	b0[3] = b[3][0];
	b1[0] = b[0][1];
	b1[1] = b[1][1];
	b1[2] = b[2][1];
	b1[3] = b[3][1];
	b2[0] = b[0][2];
	b2[1] = b[1][2];
	b2[2] = b[2][2];
	b2[3] = b[3][2];
	b3[0] = b[0][3];
	b3[1] = b[1][3];
	b3[2] = b[2][3];
	b3[3] = b[3][3];
	out[0][0] = dot4(a[0], b0);
	out[0][1] = dot4(a[0], b1);
	out[0][2] = dot4(a[0], b2);
	out[0][3] = dot4(a[0], b3);
	out[1][0] = dot4(a[1], b0);
	out[1][1] = dot4(a[1], b1);
	out[1][2] = dot4(a[1], b2);
	out[1][3] = dot4(a[1], b3);
	out[2][0] = dot4(a[2], b0);
	out[2][1] = dot4(a[2], b1);
	out[2][2] = dot4(a[2], b2);
	out[2][3] = dot4(a[2], b3);
	out[3][0] = dot4(a[3], b0);
	out[3][1] = dot4(a[3], b1);
	out[3][2] = dot4(a[3], b2);
	out[3][3] = dot4(a[3], b3);
	return out;
}

float* mulMat3Vec3(const float mat[3][3], const float vec[3], float out[3]) {
	out[0] = dot3(mat[0], vec);
	out[1] = dot3(mat[1], vec);
	out[2] = dot3(mat[2], vec);
	return out;
}

float* mulMat4Vec4(const float mat[4][4], const float vec[4], float out[4]) {
	out[0] = dot4(mat[0], vec);
	out[1] = dot4(mat[1], vec);
	out[2] = dot4(mat[2], vec);
	out[3] = dot4(mat[3], vec);
	if(out[3] != 1.0F) {
		if(out[3] == 0.0F) {
			out[0] = FLT_MAX;
			out[1] = FLT_MAX;
			out[2] = FLT_MAX;
			out[3] = FLT_MAX;
		} else {
			out[0] /= out[3];
			out[1] /= out[3];
			out[2] /= out[3];
			out[3] = 1.0F;
		}
	}
	return out;
}

float* mulMat4Vec4Proj(const float mat[4][4], const float vec[4], float out[4]) {
	out[0] = dot4(mat[0], vec);
	out[1] = dot4(mat[1], vec);
	out[2] = dot4(mat[2], vec);
	out[3] = dot4(mat[3], vec);
	if(out[2] <= 0.0F) {
		out[3] = 1.0F;
	} else {
		out[3] = 1.0F / out[3];
	}
	return out;
}

float (*transposeMat3(const float mat[3][3], float out[3][3]))[3] {
	out[0][0] = mat[0][0];
	out[1][0] = mat[0][1];
	out[2][0] = mat[0][2];
	out[0][1] = mat[1][0];
	out[1][1] = mat[1][1];
	out[2][1] = mat[1][2];
	out[0][2] = mat[2][0];
	out[1][2] = mat[2][1];
	out[2][2] = mat[2][2];
	return out;
}

float (*transposeMat4(const float mat[4][4], float out[4][4]))[4] {
	out[0][0] = mat[0][0];
	out[1][0] = mat[0][1];
	out[2][0] = mat[0][2];
	out[3][0] = mat[0][3];
	out[0][1] = mat[1][0];
	out[1][1] = mat[1][1];
	out[2][1] = mat[1][2];
	out[3][1] = mat[1][3];
	out[0][2] = mat[2][0];
	out[1][2] = mat[2][1];
	out[2][2] = mat[2][2];
	out[3][2] = mat[2][3];
	out[0][3] = mat[3][0];
	out[1][3] = mat[3][1];
	out[2][3] = mat[3][2];
	out[3][3] = mat[3][3];
	return out;
}

float (*cofactor3(const float in[3][3], int row, int col, float out[2][2]))[2] {
	int r, c;
	int cords[2] = { 0, 0 };
	for(r = 0;r < 3;r++) {
			if(r == row) continue;
			cords[1] = 0;
			for(c = 0;c < 3;c++) {
					if(c == col) continue;
					out[cords[0]][cords[1]] = in[r][c];
					cords[1] += 1;
			}
			cords[0] += 1;
	}
	if(row == 1 ^ col == 1) {
		float temp[2];
		memcpy_s(temp, sizeof(temp), out[0], sizeof(temp));
		memcpy_s(out[0], sizeof(temp), out[1], sizeof(temp));
		memcpy_s(out[1], sizeof(temp), temp, sizeof(temp));
	}
	return out;
}

float (*cofactorsMat3(const float in[3][3], float out[3][3]))[3] {
	float temp[2][2];
	out[0][0] = det2(cofactor3(in, 0, 0, temp));
	out[0][1] = det2(cofactor3(in, 0, 1, temp));
	out[0][2] = det2(cofactor3(in, 0, 2, temp));
	out[1][0] = det2(cofactor3(in, 1, 0, temp));
	out[1][1] = det2(cofactor3(in, 1, 1, temp));
	out[1][2] = det2(cofactor3(in, 1, 2, temp));
	out[2][0] = det2(cofactor3(in, 2, 0, temp));
	out[2][1] = det2(cofactor3(in, 2, 1, temp));
	out[2][2] = det2(cofactor3(in, 2, 2, temp));
	return out;
}

float (*adjugate3(float in[3][3], float out[3][3]))[3] {
	float temp[3][3];
  transposeMat3(cofactorsMat3(in, temp), out);
	return out;
}

float det2(float in[2][2]) {
	return in[0][0] * in[1][1] - in[0][1] * in[1][0];
}

float det3(float in[3][3]) {
  return in[0][0] * in[1][1] * in[2][2] + in[0][1] * in[1][2] * in[2][0] + in[0][2] * in[1][0] * in[2][1]
  	- in[0][2] * in[1][1] * in[2][0] - in[0][1] * in[1][0] * in[2][2] - in[0][0] * in[1][2] * in[2][1];
}

float det4(float in[4][4]) {
	float tempMat3[3][3];
  float a = in[0][0] * det3(cofactor4(in, 0, 0, tempMat3));
	float b = in[0][1] * det3(cofactor4(in, 0, 1, tempMat3));
	float c = in[0][2] * det3(cofactor4(in, 0, 2, tempMat3));
	float d = in[0][3] * det3(cofactor4(in, 0, 3, tempMat3));
  return a - b + c - d;
}

float (*inverse3(float in[3][3], float out[3][3]))[3] {
    float det = det3(in);
    float adjugate[3][3];
		adjugate3(in, adjugate);
		divMat3ByScalar(adjugate, det, out);
		return out;
}

float (*cofactor4(const float in[4][4], int row, int col, float out[3][3]))[3] {
	int r, c;
	int cords[2] = { 0, 0 };
	for(r = 0;r < 4;r++) {
			if(r == row) continue;
			cords[1] = 0;
			for(c = 0;c < 4;c++) {
					if(c == col) continue;
					out[cords[0]][cords[1]] = in[r][c];
					cords[1] += 1;
			}
			cords[0] += 1;
	}
	return out;
}

float (*cofactorsMat4(const float in[4][4], float out[4][4]))[4] {
	float temp[3][3];
	int r, c;
	for(r = 0;r < 4;r++) {
		for(c = 0;c < 4;c++) {
			out[r][c] = det3(cofactor4(in, r, c, temp));
			if(r % 2 ^ c % 2) out[r][c] *= -1.0F;
		}
	}
	return out;
}

float (*adjugate4(float in[4][4], float out[4][4]))[4] {
	float temp[4][4];
  transposeMat4(cofactorsMat4(in, temp), out);
	return out;
}

float (*inverse4(float in[4][4], float out[4][4]))[4] {
    float det = det4(in);
    float adjugate[4][4];
		adjugate4(in, adjugate);
		divMat4ByScalar(adjugate, det, out);
		return out;
}

static float* orthogonalizeProjection3(float project[3], float projected[3], float out[3]) {
	float scalar;
	float scalarNumerator = dot3(project, projected);
	float scalarDenominator = dot3(projected, projected);
	if(scalarDenominator == 0.0F) {
		clearVec3(out);
		return out;
	}
	scalar = scalarNumerator / scalarDenominator;
	mulVec3ByScalar(projected, scalar, out);
	return out;
}

float (*orthogonalize3(float in[3][3], float out[3][3]))[3] {
	float temp[4][3];
	normalize3(in[0], out[0]);
	subVec3(in[1], orthogonalizeProjection3(in[1], in[0], temp[0]), temp[1]);
	normalize3(temp[1], out[1]);
	subVec3(in[2], subVec3(orthogonalizeProjection3(in[2], temp[1], temp[0]), orthogonalizeProjection3(in[2], in[0], temp[1]), temp[2]), temp[3]);
	normalize3(temp[3], out[2]);
	return out;
}

float *getTriangleCM3(float triangle[3][3], float out[3]) {
	float temp[3];
	divVec3ByScalar(addVec3(addVec3(triangle[0], triangle[1], temp), triangle[2], temp), 3.0F, out);
	return out;
}

float (*genSkewMat3(float in[3], float out[3][3]))[3] {
	memset(out, 0, 9 * sizeof(float));
	out[0][1] = - in[2];
	out[0][2] = in[1];
	out[1][0] = in[2];
	out[1][2] = - in[0];
	out[2][0] = - in[1];
	out[2][1] = in[0];
	return out;
}

float (*genIdentityMat3(float mat[3][3]))[3] {
  mat[0][0] = 1.0;
  mat[0][1] = 0.0;
	mat[0][2] = 0.0;
  mat[1][0] = 0.0;
  mat[1][1] = 1.0;
	mat[1][2] = 0.0;
	mat[2][0] = 0.0;
	mat[2][1] = 0.0;
	mat[2][2] = 1.0;
  return mat;
}

float (*genIdentityMat4(float mat[4][4]))[4] {
  mat[0][0] = 1.0;
  mat[0][1] = 0.0;
	mat[0][2] = 0.0;
	mat[0][3] = 0.0;
  mat[1][0] = 0.0;
  mat[1][1] = 1.0;
	mat[1][2] = 0.0;
	mat[1][3] = 0.0;
	mat[2][0] = 0.0;
	mat[2][1] = 0.0;
	mat[2][2] = 1.0;
	mat[2][3] = 0.0;
	mat[3][0] = 0.0;
	mat[3][1] = 0.0;
	mat[3][2] = 0.0;
	mat[3][3] = 1.0;
  return mat;
}

float (*genTranslationMat3(float dx, float dy, float mat[3][3]))[3] {
	mat[0][0] = 1.0;
  mat[0][1] = 0.0;
	mat[0][2] = dx;
  mat[1][0] = 0.0;
  mat[1][1] = 1.0;
	mat[1][2] = dy;
	mat[2][0] = 0.0;
	mat[2][1] = 0.0;
	mat[2][2] = 1.0;
	return mat;
}

float (*genTranslationMat4(float dx, float dy, float dz, float mat[4][4]))[4] {
	mat[0][0] = 1.0;
  mat[0][1] = 0.0;
	mat[0][2] = 0.0;
	mat[0][3] = dx;
  mat[1][0] = 0.0;
  mat[1][1] = 1.0;
	mat[1][2] = 0.0;
	mat[1][3] = dy;
	mat[2][0] = 0.0;
	mat[2][1] = 0.0;
	mat[2][2] = 1.0;
	mat[2][3] = dz;
	mat[3][0] = 0.0;
	mat[3][1] = 0.0;
	mat[3][2] = 0.0;
	mat[3][3] = 1.0;
	return mat;
}

float (*genScaleMat3(float sx, float sy, float mat[3][3]))[3] {
  mat[0][0] = sx;
  mat[0][1] = 0.0;
	mat[0][2] = 0.0;
  mat[1][0] = 0.0;
  mat[1][1] = sy;
	mat[1][2] = 0.0;
	mat[2][0] = 0.0;
	mat[2][1] = 0.0;
	mat[2][2] = 1.0;
  return mat;
}

float (*genScaleMat4(float sx, float sy, float sz, float mat[4][4]))[4] {
  mat[0][0] = sx;
  mat[0][1] = 0.0;
	mat[0][2] = 0.0;
	mat[0][3] = 0.0;
  mat[1][0] = 0.0;
  mat[1][1] = sy;
	mat[1][2] = 0.0;
	mat[1][3] = 0.0;
	mat[2][0] = 0.0;
	mat[2][1] = 0.0;
	mat[2][2] = sz;
	mat[2][3] = 0.0;
	mat[3][0] = 0.0;
	mat[3][1] = 0.0;
	mat[3][2] = 0.0;
	mat[3][3] = 1.0;
  return mat;
}

float (*genRotationMat3(float rotation, float mat[3][3]))[3] {
	mat[0][0] = cosf(rotation);
	mat[0][1] = -sinf(rotation);
	mat[0][2] = 0.0;
	mat[1][0] = sinf(rotation);
	mat[1][1] = cosf(rotation);
	mat[1][2] = 0.0;
	mat[2][0] = 0.0;
	mat[2][1] = 0.0;
	mat[2][2] = 1.0;
	return mat;
}

float (*genRotationXMat4(float rotation, float mat[4][4]))[4] {
	mat[0][0] = 1.0;
	mat[0][1] = 0.0;
	mat[0][2] = 0.0;
	mat[0][3] = 0.0;
	mat[1][0] = 0.0;
	mat[1][1] = cosf(rotation);
	mat[1][2] = -sinf(rotation);
	mat[1][3] = 0.0;
	mat[2][0] = 0.0;
	mat[2][1] = sinf(rotation);
	mat[2][2] = cosf(rotation);
	mat[2][3] = 0.0;
	mat[3][0] = 0.0;
	mat[3][1] = 0.0;
	mat[3][2] = 0.0;
	mat[3][3] = 1.0;
	return mat;
}

float (*genRotationYMat4(float rotation, float mat[4][4]))[4] {
	mat[0][0] = cosf(rotation);
	mat[0][1] = 0.0;
	mat[0][2] = sinf(rotation);
	mat[0][3] = 0.0;
	mat[1][0] = 0.0;
	mat[1][1] = 1.0;
	mat[1][2] = 0.0;
	mat[1][3] = 0.0;
	mat[2][0] = -sinf(rotation);
	mat[2][1] = 0.0;
	mat[2][2] = cosf(rotation);
	mat[2][3] = 0.0;
	mat[3][0] = 0.0;
	mat[3][1] = 0.0;
	mat[3][2] = 0.0;
	mat[3][3] = 1.0;
	return mat;
}

float (*genRotationZMat4(float rotation, float mat[4][4]))[4] {
	mat[0][0] = cosf(rotation);
	mat[0][1] = -sinf(rotation);
	mat[0][2] = 0.0;
	mat[0][3] = 0.0;
	mat[1][0] = sinf(rotation);
	mat[1][1] = cosf(rotation);
	mat[1][2] = 0.0;
	mat[1][3] = 0.0;
	mat[2][0] = 0.0;
	mat[2][1] = 0.0;
	mat[2][2] = 1.0;
	mat[2][3] = 0.0;
	mat[3][0] = 0.0;
	mat[3][1] = 0.0;
	mat[3][2] = 0.0;
	mat[3][3] = 1.0;
	return mat;
}

float (*genRotationMat4(float rx, float ry, float rz, float mat[4][4]))[4] {
	float xMat[4][4];
	float yMat[4][4];
	float zMat[4][4];
	float temp[4][4];
	genRotationXMat4(rx, xMat);
	genRotationYMat4(ry, yMat);
	genRotationZMat4(rz, zMat);
	return mulMat4(mulMat4(zMat, yMat, temp), xMat, mat);
}

float *getAngleFromMat3(float in[3][3], float out[3]) {
	out[0] = atan2(in[2][1], in[2][2]);
	out[1] = atan2(-in[2][0], sqrt(in[2][1] * in[2][1] + in[2][2] * in[2][2]));
	out[2] = atan2(in[1][0], in[0][0]);
	return out;
}

float (*genLookAtMat4(float position[3], float target[3], float worldUp[3], float mat[4][4]))[4] {
	float direction[3];
	float right[3];
	float up[3];
	float look[4][4] = { { 0.0F } };
	float move[4][4] = { { 0.0F } };
	normalize3(subVec3(position, target, direction), direction);
	normalize3(cross(direction, worldUp, right), right);
	cross(direction, right, up);
	look[0][0] = right[0];
	look[0][1] = right[1];
	look[0][2] = right[2];
	look[1][0] = up[0];
	look[1][1] = up[1];
	look[1][2] = up[2];
	look[2][0] = direction[0];
	look[2][1] = direction[1];
	look[2][2] = direction[2];
	look[3][3] = 1.0F;
	move[0][0] = 1.0F;
	move[0][3] = -position[0];
	move[1][1] = 1.0F;
	move[1][3] = -position[1];
	move[2][2] = 1.0F;
	move[2][3] = -position[2];
	move[3][3] = 1.0F;
	return mulMat4(look, move, mat);
}

float (*genPerspectiveMat4(float fovY, float zNear, float zFar, float aspect, float mat[4][4]))[4] {
	float top = zNear * tanf(fovY / 2.0F);
	float right = top * aspect;
	mat[0][0] = zNear / right;
	mat[0][1] = 0.0F;
	mat[0][2] = 0.0F;
	mat[0][3] = 0.0F;
	mat[1][0] = 0.0F;
	mat[1][1] = zNear / top;
	mat[1][2] = 0.0F;
	mat[1][3] = 0.0F;
	mat[2][0] = 0.0F;
	mat[2][1] = 0.0F;
	mat[2][2] = - (zFar + zNear) / (zFar - zNear);
	mat[2][3] = -2.0F * zFar * zNear / (zFar - zNear);
	mat[3][0] = 0.0F;
	mat[3][1] = 0.0F;
	mat[3][2] = -1.0F;
	mat[3][3] = 0.0F;
	return mat;
}

float (*genInertiaTensorBox(float mass, float width, float height, float depth, float out[3][3]))[3] {
	float pre = mass / 12.0F;
	memset(out, 0, 9 * sizeof(float));
	out[0][0] = pre * (height * height + depth * depth);
	out[1][1] = pre * (width * width + depth * depth);
	out[2][2] = pre * (width * width + height * height);
	return out;
}

// float (*genInertiaTensorCylinder(float mass, float radius, float height, float out[3][3]))[3] {
// 	float pre = mass / 12.0F * (3.0F * radius * radius + height * height);
// 	memset(out, 0, 9 * sizeof(float));
// 	out[0][0] = pre;
// 	out[1][1] = pre;
// 	out[2][2] = mass / 2.0F * ();
// 	return out;
// }

void printVec3(float vec[3]) {
	int i;
	printf("(");
	for(i = 0;i < 3;i++) {
		printf("%10f", (double)vec[i]);
		if(i != 2) printf(", ");
	}
	printf(")\n");
}

void printVec4(float vec[4]) {
	int i;
	printf("(");
	for(i = 0;i < 4;i++) {
		printf("%10f", (double)vec[i]);
		if(i != 3) printf(", ");
	}
	printf(")\n");
}

void printMat3(float mat[3][3]) {
	int row, col;
	for(row = 0;row < 3;row++) {
		printf("| ");
		for(col = 0;col < 3;col++) printf("%10f ", (double)mat[row][col]);
		printf("|\n");
	}
	printf("\n");
}

void printMat4(float mat[4][4]) {
	int row, col;
	for(row = 0;row < 4;row++) {
		printf("| ");
		for(col = 0;col < 4;col++) printf("%10f ", (double)mat[row][col]);
		printf("|\n");
	}
	printf("\n");
}
