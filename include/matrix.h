/**
* @file matrix.h
* \~english @brief Linear Algebra
* \~japanese @brief ���`�n���Z�֐�
* \~english @details float[n] are used as n dimensional vectors.
* \~japanese @details Float�z�񂪃x�N�g���Ƃ��ĉ��߂���܂��B
*/

#ifndef MATRIX_H
#define MATRIX_H

/**
* \~english @brief Pi.
* \~japanese @brief �~�����B
*/
#define PI  3.14159265359F

/**
* \~english @brief Returns the sign of the argument.
* \~japanese @brief �����̕�����Ԃ��B
*/
#define sign(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))

/**
* \~english @brief Size of two dimensional vector.
* \~japanese @brief �񎟌��x�N�g���̃T�C�Y�B
*/
#define SIZE_VEC2 (2 * sizeof(float))
/**
* \~english @brief Size of three dimensional vector.
* \~japanese @brief �O�����x�N�g���̃T�C�Y�B
*/
#define SIZE_VEC3 (3 * sizeof(float))
/**
* \~english @brief Size of four dimensional vector.
* \~japanese @brief �l�����x�N�g���̃T�C�Y�B
*/
#define SIZE_VEC4 (4 * sizeof(float))
/**
* \~english @brief Size of four dimensional squared matrix.
* \~japanese @brief �l���������s��̃T�C�Y�B
*/
#define SIZE_MAT4 (16 * sizeof(float))

/**
* \~english @brief A mask that specifies x element of a vector.
* \~japanese @brief �x�N�g����X�v�f��I������r�b�g�}�X�N�B
*/
#define X_MASK 1
/**
* \~english @brief A mask that specifies y element of a vector.�@
* \~japanese @brief �x�N�g����Y�v�f��I������r�b�g�}�X�N�B
*/
#define Y_MASK 2
/**
* \~english @brief A mask that specifies x and y elements of a vector.
* \~japanese @brief �x�N�g����XY�v�f��I������r�b�g�}�X�N�B
*/
#define XY_MASK 3
/**
* \~english @brief A mask that specifies z element of a vector.
* \~japanese @brief �x�N�g����Z�v�f��I������r�b�g�}�X�N�B
*/
#define Z_MASK 4
/**
* \~english @brief A mask that specifies x and z elements of a vector.
* \~japanese @brief �x�N�g����XZ�v�f��I������r�b�g�}�X�N�B
*/
#define XZ_MASK 5
/**
* \~english @brief A mask that specifies y and z elements of a vector.
* \~japanese @brief �x�N�g����YZ�v�f��I������r�b�g�}�X�N�B
*/
#define YZ_MASK 6
/**
* \~english @brief A mask that specifies x, y and z elements of a vector.
* \~japanese @brief �x�N�g����XYZ�v�f��I������r�b�g�}�X�N�B
*/
#define XYZ_MASK 7

/**
* \~english @brief Print two dimensional vector to stdout.
* \~japanese @brief �񎟌��x�N�g����W���o�͂ɏo�͂���B
*/
#define printVec2(vec) printf("(%10f, %10f)\n", (double)(vec)[0], (double)(vec)[1])
/**
* \~english @brief Print three dimensional vector to stdout.
* \~japanese @brief �O�����x�N�g����W���o�͂ɏo�͂���B
*/
#define printVec3(vec) printf("(%10f, %10f, %10f)\n", (double)(vec)[0], (double)(vec)[1], (double)(vec)[2])
/**
* \~english @brief Print four dimensional vector to stdout.
* \~japanese @brief �l�����x�N�g����W���o�͂ɏo�͂���B
*/
#define printVec4(vec) printf("(%10f, %10f, %10f, %10f)\n", (double)(vec)[0], (double)(vec)[1], (double)(vec)[2], (double)(vec)[3])

/**
* \~english @brief Clears a two dimensional vector.
* \~japanese @brief �񎟌��x�N�g�����N���A����B
*/
#define clearVec2(vec) memset(vec, 0, SIZE_VEC2)
/**
* \~english @brief Clears a three dimensional vector.
* \~japanese @brief �O�����x�N�g�����N���A����B
*/
#define clearVec3(vec) memset(vec, 0, SIZE_VEC3)
/**
* \~english @brief Clears a four dimensional vector.
* \~japanese @brief �l�����x�N�g�����N���A����B
*/
#define clearVec4(vec) memset(vec, 0, SIZE_VEC4)

/**
* \~english @brief Copys a two dimensional vector.
* \~japanese @brief �񎟌��x�N�g�����R�s�[����B
*/
#define copyVec2(dest, src) memcpy_s(dest, SIZE_VEC2, src, SIZE_VEC2)
/**
* \~english @brief Copys a three dimensional vector.
* \~japanese @brief �O�����x�N�g�����R�s�[����B
*/
#define copyVec3(dest, src) memcpy_s(dest, SIZE_VEC3, src, SIZE_VEC3)
/**
* \~english @brief Copys a four dimensional vector.
* \~japanese @brief �l�����x�N�g�����R�s�[����B
*/
#define copyVec4(dest, src) memcpy_s(dest, SIZE_VEC4, src, SIZE_VEC4)

/**
* \~english @brief Converts a three dimensional vector to a four dimensional vector.
* \~japanese @brief �O�����x�N�g�����l�����x�N�g���֕ϊ�����B
*/
float *convVec3toVec4(const float in[3], float out[4]);
/**
* \~english @brief Converts a four dimensional vector to a three dimensional vector.
* \~japanese @brief �l�����x�N�g�����O�����x�N�g���֕ϊ�����B
*/
float *convVec4toVec3(const float in[4], float out[3]);
/**
* \~english @brief Extracts components from a three dimensional vector.
* \~japanese @brief �O�����x�N�g������v�f�𒊏o����B
*/
float *extractComponents3(const float in[3], int mask, float out[3]);

/**
* \~english @brief Converts a four squared dimensional matrix to a three squared dimensional matrix.
* \~japanese @brief �l���������s����O���������s��֕ϊ�����B
*/
float (*convMat4toMat3(float in[4][4], float out[3][3]))[3];

/**
* \~english @brief Initializes a 3D vector. Components specified by the mask are set to 1.
* \~japanese @brief �O�����x�N�g�������������܂��B�}�X�N�Ŏw�肳�ꂽ�v�f�͂P�ɃZ�b�g����܂��B
*/
float* initVec3(float vec[3], int mask);
/**
* \~english @brief Set a value for components specified by the mask.
* \~japanese @brief �}�X�N�Ŏw�肳�ꂽ�v�f�ɒl���Z�b�g���܂��B
*/
float* setVec3(float vec[3], float val, int mask);
/**
* \~english @brief Checks wheather two 3d vectors are equal or not.
* \~japanese @brief ��̎O�����x�N�g�������������ǂ������`�F�b�N���܂��B
*/
int equalVec3(float a[3], float b[3]);
/**
* \~english @brief Returns the index of 3d vector that has the maximum absolute value.
* \~japanese @brief �O�����x�N�g���̐�Βl���ő�ȃC���f�b�N�X��Ԃ��܂��B
*/
int maxAbsIndex3(float vector[3]);
/**
* \~english @brief Returns cos() of two 3d vectors.
* \~japanese @brief �Q�̎O�����x�N�g���̃R�T�C����Ԃ��܂��B
*/
float cosVec3(float a[3], float b[3]);

/**
* \~english @brief Returns the dot of two 2d vectors.
* \~japanese @brief �񎟌��x�N�g���̓��ς�Ԃ��܂��B
*/
float dot2(const float a[2], const float b[2]);
/**
* \~english @brief Returns the dot of two 3d vectors.
* \~japanese @brief �R�����x�N�g���̓��ς�Ԃ��܂��B
*/
float dot3(const float a[3], const float b[3]);
/**
* \~english @brief Returns the dot of two 4d vectors.
* \~japanese @brief �S�����x�N�g���̓��ς�Ԃ��܂��B
*/
float dot4(const float a[4], const float b[4]);
/**
* \~english @brief Returns the cross of two 3d vectors.
* \~japanese @brief �R�����x�N�g���̊O�ς�Ԃ��܂��B
*/
float* cross(const float a[3], const float b[3], float out[3]);
/**
* \~english @brief Returns the area of the triangle.
* \~japanese @brief �O�p�`�̖ʐς�Ԃ��܂��B
*/
float areaOfTriangle(float triangle[3][3]);
/**
* \~english @brief Returns the magnitude of the 2d vector.
* \~japanese @brief �񎟌��x�N�g���̑傫����Ԃ��܂��B
*/
float length2(const float vector[2]);
/**
* \~english @brief Returns the magnitude of the 3d vector.
* \~japanese @brief �R�����x�N�g���̑傫����Ԃ��܂��B
*/
float length3(const float vector[3]);
/**
* \~english @brief Returns the distance between two 2d coordinates.
* \~japanese @brief ��_�̓񎟌����W�̋�����Ԃ��܂��B
*/
float distance2(const float a[2], const float b[2]);
/**
* \~english @brief Returns the distance between two 3d coordinates.
* \~japanese @brief ��_�̂R�������W�̋�����Ԃ��܂��B
*/
float distance3(const float a[3], const float b[3]);
/**
* \~english @brief Add two 2d vectors.
* \~japanese @brief ��̓񎟌��x�N�g���𑫂��܂��B
*/
float* addVec2(const float a[2], const float b[2], float out[2]);
/**
* \~english @brief Add two 3d vectors.
* \~japanese @brief ��̂R�����x�N�g���𑫂��܂��B
*/
float* addVec3(const float a[3], const float b[3], float out[3]);
/**
* \~english @brief Substracts a 2d vector from another 2d vector.
* \~japanese @brief ����񎟌��x�N�g������ق��̓񎟌��x�N�g���������܂��B
*/
float* subVec2(const float a[2], const float b[2], float out[2]);
/**
* \~english @brief Substracts a 3d vector from another 3d vector.
* \~japanese @brief ����R�����x�N�g������ق��̂R�����x�N�g���������܂��B
*/
float* subVec3(const float a[3], const float b[3], float out[3]);
/**
* \~english @brief Multiples a 2d vector by a scalar.
* \~japanese @brief �񎟌��x�N�g�����X�J���[�{���܂��B
*/
float* mulVec2ByScalar(const float vector[2], float scalar, float out[2]);
/**
* \~english @brief Divides a 2d vector by a scalar.
* \~japanese @brief �񎟌��x�N�g�����X�J���[�Ŋ���܂��B
*/
float* divVec2ByScalar(const float vector[2], float scalar, float out[2]);
/**
* \~english @brief Multiples a 3d vector by a scalar.
* \~japanese @brief �R�����x�N�g�����X�J���[�{���܂��B
*/
float* mulVec3ByScalar(const float vector[3], float scalar, float out[3]);
/**
* \~english @brief Divides a 3d vector by a scalar.
* \~japanese @brief �R�����x�N�g�����X�J���[�Ŋ���܂��B
*/
float* divVec3ByScalar(const float vector[3], float scalar, float out[3]);
/**
* \~english @brief Multiples a 4d vector by a scalar.
* \~japanese @brief �S�����x�N�g�����X�J���[�{���܂��B
*/
float* mulVec4ByScalar(const float vector[4], float scalar, float out[4]);
/**
* \~english @brief Divides a 4d vector by a scalar.
* \~japanese @brief �S�����x�N�g�����X�J���[�Ŋ���܂��B
*/
float* divVec4ByScalar(const float vector[4], float scalar, float out[4]);
/**
* \~english @brief Normalize a 2d vector.
* \~japanese @brief �񎟌��x�N�g���𐳋K�����܂��B
*/
float* normalize2(const float vector[2], float out[2]);
/**
* \~english @brief Normalize a 3d vector.
* \~japanese @brief �R�����x�N�g���𐳋K�����܂��B
*/
float* normalize3(const float vector[3], float out[3]);
/**
* \~english @brief Calculates the angle between two 2d vectors.
* \~japanese @brief �Q�̓񎟌��x�N�g���Ԃ̊p�x�����߂܂��B
*/
float* direction2(const float a[2], const float b[2], float out[2]);
/**
* \~english @brief Calculates the angle between two 3d vectors.
* \~japanese @brief �Q�̂R�����x�N�g���Ԃ̊p�x�����߂܂��B
*/
float* direction3(const float a[3], const float b[3], float out[3]);
/**
* \~english @brief Calculates the angle of the 2d vector.
* \~japanese @brief �񎟌��x�N�g���̊p�x�����߂܂��B
*/
float angleVec2(const float vector[2]);

/**
* \~english @brief Adds two 3d squared matrices.
* \~japanese @brief �Q�̎O���������s��𑫂��܂��B
*/
float (*addMat3(const float a[3][3], const float b[3][3], float out[3][3]))[3];
/**
* \~english @brief Multiplies two 3d squared matrices.
* \~japanese @brief �Q�̎O���������s����|���܂��B
*/
float	(*mulMat3(const float a[3][3], const float b[3][3], float out[3][3]))[3];
/**
* \~english @brief Multiplies two 4d squared matrices.
* \~japanese @brief �Q�̂S���������s����|���܂��B
*/
float	(*mulMat4(const float a[4][4], const float b[4][4], float out[4][4]))[4];
/**
* \~english @brief Multiplies a 3d squared matrix by a scalar.
* \~japanese @brief �O���������s��ɃX�J���[���|���܂��B
*/
float	(*mulMat3ByScalar(const float in[3][3], float scalar, float out[3][3]))[3];
/**
* \~english @brief Divdes a 3d squared matrix by a scalar.
* \~japanese @brief �O���������s����X�J���[�Ŋ���܂��B
*/
float	(*divMat3ByScalar(const float in[3][3], float scalar, float out[3][3]))[3];
/**
* \~english @brief Divdes a 4d squared matrix by a scalar.
* \~japanese @brief �S���������s����X�J���[�Ŋ���܂��B
*/
float	(*divMat4ByScalar(const float in[4][4], float scalar, float out[4][4]))[4];
/**
* \~english @brief Multiplies a 3d squared matrix by a 3d vector.
* \~japanese @brief �O���������s���3�����x�N�g�����|���܂��B
*/
float* mulMat3Vec3(const float mat[3][3], const float vec[3], float out[3]);
/**
* \~english @brief Multiplies a 4d squared matrix by a 4d vector.
* \~japanese @brief �S���������s��ɂS�����x�N�g�����|���܂��B
*/
float* mulMat4Vec4(const float mat[4][4], const float vec[4], float out[4]);
/**
* \~english @brief Multiplies a 4d squared matrix by a 4d vector. (For projection usage.)
* \~japanese @brief �S���������s��ɂS�����x�N�g�����|���܂��B�i���e�p�j
*/
float* mulMat4Vec4Proj(const float mat[4][4], const float vec[4], float out[4]);
/**
* \~english @brief Transposes a 3d squared matrix.
* \~japanese @brief �R���������s���]�u���܂��B
*/
float (*transposeMat3(const float mat[3][3], float out[3][3]))[3];
/**
* \~english @brief Transposes a 4d squared matrix.
* \~japanese @brief �S���������s���]�u���܂��B
*/
float (*transposeMat4(const float mat[4][4], float out[4][4]))[4];

/**
* \~english @brief Calculates a cofactor of a 3d squared matrix.
* \~japanese @brief �R���������s��̗]���q�����߂܂��B
*/
float (*cofactor3(const float in[3][3], int row, int col, float out[2][2]))[2];
/**
* \~english @brief Calculates a cofactor of a 4d squared matrix.
* \~japanese @brief �S���������s��̗]���q�����߂܂��B
*/
float (*cofactor4(const float in[4][4], int row, int col, float out[3][3]))[3];
/**
* \~english @brief Calculates a cofactor matrix of a 3d squared matrix.
* \~japanese @brief �R���������s��̗]���q�̍s������߂܂��B
*/
float (*cofactorsMat3(const float in[3][3], float out[3][3]))[3];
/**
* \~english @brief Calculates a cofactor matrix of a 4d squared matrix.
* \~japanese @brief �S���������s��̗]���q�̍s������߂܂��B
*/
float (*cofactorsMat4(const float in[4][4], float out[4][4]))[4];
/**
* \~english @brief Calculates an adjugate matrix of a 3d squared matrix.
* \~japanese @brief �R���������s��̗]���q�s������߂܂��B
*/
float (*adjugate3(float in[3][3], float out[3][3]))[3];
/**
* \~english @brief Calculates an adjugate matrix of a 4d squared matrix.
* \~japanese @brief �S���������s��̗]���q�s������߂܂��B
*/
float (*adjugate4(float in[4][4], float out[4][4]))[4];
/**
* \~english @brief Calculates the determinant of the 2d squared matrix.
* \~japanese @brief �Q���������s��̍s�񎮂̒l�����߂܂��B
*/
float det2(float in[2][2]);
/**
* \~english @brief Calculates the determinant of the 3d squared matrix.
* \~japanese @brief �R���������s��̍s�񎮂̒l�����߂܂��B
*/
float det3(float in[3][3]);
/**
* \~english @brief Calculates the determinant of the 4d squared matrix.
* \~japanese @brief �S���������s��̍s�񎮂̒l�����߂܂��B
*/
float det4(float in[4][4]);
/**
* \~english @brief Calculates the inverse of the 3d squared matrix.
* \~japanese @brief �R���������s��̋t�s������߂܂��B
*/
float (*inverse3(float in[3][3], float out[3][3]))[3];
/**
* \~english @brief Calculates the inverse of the 4d squared matrix.
* \~japanese @brief �S���������s��̋t�s������߂܂��B
*/
float (*inverse4(float in[4][4], float out[4][4]))[4];
/**
* \~english @brief Orthogonalize a 3d squared matrix.
* \~japanese @brief �R���������s��𒼍s�����܂��B
*/
float (*orthogonalize3(float in[3][3], float out[3][3]))[3];

/**
* \~english @brief Generates the skew of the 3d squared matrix.
* \~japanese @brief �R���������s��̃X�L���[�𐶐����܂��B
*/
float (*genSkewMat3(float in[3], float out[3][3]))[3];
/**
* \~english @brief Generates a translate matrix in a 3d squared matrix.
* \~japanese @brief �ړ����R���������s��Ő������܂��B
*/
float (*genTranslationMat3(float dx, float dy, float mat[3][3]))[3];
/**
* \~english @brief Generates a translate matrix in a 4d squared matrix.
* \~japanese @brief �ړ����S���������s��Ő������܂��B
*/
float (*genTranslationMat4(float dx, float dy, float dz, float mat[4][4]))[4];
/**
* \~english @brief Generates a 3d squared identiry matrix.
* \~japanese @brief �R���������P�ʍs��𐶐����܂��B
*/
float (*genIdentityMat3(float mat[3][3]))[3];
/**
* \~english @brief Generates a 4d squared identiry matrix.
* \~japanese @brief �S���������P�ʍs��𐶐����܂��B
*/
float (*genIdentityMat4(float mat[4][4]))[4];
/**
* \~english @brief Generates a 3d squared scale matrix.
* \~japanese @brief �X�P�[���̂R���������s��𐶐����܂��B
*/
float (*genScaleMat3(float sx, float sy, float mat[3][3]))[3];
/**
* \~english @brief Generates a 4d squared scale matrix.
* \~japanese @brief �X�P�[���̂S���������s��𐶐����܂��B
*/
float (*genScaleMat4(float sx, float sy, float sz, float mat[4][4]))[4];
/**
* \~english @brief Generates a 3d squared rotation matrix.
* \~japanese @brief ��]�̂R���������s��𐶐����܂��B
*/
float (*genRotationMat3(float rotation, float mat[3][3]))[3];
/**
* \~english @brief Generates a 4d squared rotation matrix with respect to x axis.
* \~japanese @brief x���ɂ��Ẳ�]�̂S���������s��𐶐����܂��B
*/
float (*genRotationXMat4(float rotation, float mat[4][4]))[4];
/**
* \~english @brief Generates a 4d squared rotation matrix with respect to y axis.
* \~japanese @brief �����ɂ��Ẳ�]�̂S���������s��𐶐����܂��B
*/
float (*genRotationYMat4(float rotation, float mat[4][4]))[4];
/**
* \~english @brief Generates a 4d squared rotation matrix with respect to z axis.
* \~japanese @brief �����ɂ��Ẳ�]�̂S���������s��𐶐����܂��B
*/
float (*genRotationZMat4(float rotation, float mat[4][4]))[4];
/**
* \~english @brief Generates a 4d squared rotation matrix.
* \~japanese @brief ��]�̂S���������s��𐶐����܂��B
*/
float (*genRotationMat4(float rx, float ry, float rz, float mat[4][4]))[4];

/**
* \~english @brief Calculates the angles of the 3d squared matrix.
* \~japanese @brief �O���������s��̊p�x���擾���܂��B
*/
float *getAngleFromMat3(float in[3][3], float out[3]);

/**
* \~english @brief Generates a look-at matrix.
* \~japanese @brief ���b�N�A�g�s��𐶐����܂��B
*/
float (*genLookAtMat4(float position[3], float target[3], float worldUp[3], float mat[4][4]))[4];
/**
* \~english @brief Generates a perspective matrix.
* \~japanese @brief ���e�s��𐶐����܂��B
*/
float (*genPerspectiveMat4(float fovY, float zNear, float zFar, float aspect, float mat[4][4]))[4];

/**
* \~english @brief Generates a inertia matrix for box.
* \~japanese @brief ���̊������[�����g�𐶐����܂��B
*/
float (*genInertiaTensorBox(float mass, float width, float height, float depth, float out[3][3]))[3];

/**
* \~english @brief Prints a 3d squared matrix to stdout.
* \~japanese @brief �O���������s���W���o�͂ɏo�͂��܂��B
*/
void printMat3(float mat[3][3]);
/**
* \~english @brief Prints a 4d squared matrix to stdout.
* \~japanese @brief �S���������s���W���o�͂ɏo�͂��܂��B
*/
void printMat4(float mat[4][4]);

#endif
