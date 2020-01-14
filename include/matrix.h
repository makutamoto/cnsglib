/**
* @file matrix.h
* \~english @brief Linear Algebra
* \~japanese @brief 線形系演算関数
* \~english @details float[n] are used as n dimensional vectors.
* \~japanese @details Float配列がベクトルとして解釈されます。
*/

#ifndef MATRIX_H
#define MATRIX_H

/**
* \~english @brief Pi.
* \~japanese @brief 円周率。
*/
#define PI  3.14159265359F

/**
* \~english @brief Returns the sign of the argument.
* \~japanese @brief 引数の符号を返す。
*/
#define sign(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))

/**
* \~english @brief Size of two dimensional vector.
* \~japanese @brief 二次元ベクトルのサイズ。
*/
#define SIZE_VEC2 (2 * sizeof(float))
/**
* \~english @brief Size of three dimensional vector.
* \~japanese @brief 三次元ベクトルのサイズ。
*/
#define SIZE_VEC3 (3 * sizeof(float))
/**
* \~english @brief Size of four dimensional vector.
* \~japanese @brief 四次元ベクトルのサイズ。
*/
#define SIZE_VEC4 (4 * sizeof(float))
/**
* \~english @brief Size of four dimensional squared matrix.
* \~japanese @brief 四次元正方行列のサイズ。
*/
#define SIZE_MAT4 (16 * sizeof(float))

/**
* \~english @brief A mask that specifies x element of a vector.
* \~japanese @brief ベクトルのX要素を選択するビットマスク。
*/
#define X_MASK 1
/**
* \~english @brief A mask that specifies y element of a vector.　
* \~japanese @brief ベクトルのY要素を選択するビットマスク。
*/
#define Y_MASK 2
/**
* \~english @brief A mask that specifies x and y elements of a vector.
* \~japanese @brief ベクトルのXY要素を選択するビットマスク。
*/
#define XY_MASK 3
/**
* \~english @brief A mask that specifies z element of a vector.
* \~japanese @brief ベクトルのZ要素を選択するビットマスク。
*/
#define Z_MASK 4
/**
* \~english @brief A mask that specifies x and z elements of a vector.
* \~japanese @brief ベクトルのXZ要素を選択するビットマスク。
*/
#define XZ_MASK 5
/**
* \~english @brief A mask that specifies y and z elements of a vector.
* \~japanese @brief ベクトルのYZ要素を選択するビットマスク。
*/
#define YZ_MASK 6
/**
* \~english @brief A mask that specifies x, y and z elements of a vector.
* \~japanese @brief ベクトルのXYZ要素を選択するビットマスク。
*/
#define XYZ_MASK 7

/**
* \~english @brief Print two dimensional vector to stdout.
* \~japanese @brief 二次元ベクトルを標準出力に出力する。
*/
#define printVec2(vec) printf("(%10f, %10f)\n", (double)(vec)[0], (double)(vec)[1])
/**
* \~english @brief Print three dimensional vector to stdout.
* \~japanese @brief 三次元ベクトルを標準出力に出力する。
*/
#define printVec3(vec) printf("(%10f, %10f, %10f)\n", (double)(vec)[0], (double)(vec)[1], (double)(vec)[2])
/**
* \~english @brief Print four dimensional vector to stdout.
* \~japanese @brief 四次元ベクトルを標準出力に出力する。
*/
#define printVec4(vec) printf("(%10f, %10f, %10f, %10f)\n", (double)(vec)[0], (double)(vec)[1], (double)(vec)[2], (double)(vec)[3])

/**
* \~english @brief Clears a two dimensional vector.
* \~japanese @brief 二次元ベクトルをクリアする。
*/
#define clearVec2(vec) memset(vec, 0, SIZE_VEC2)
/**
* \~english @brief Clears a three dimensional vector.
* \~japanese @brief 三次元ベクトルをクリアする。
*/
#define clearVec3(vec) memset(vec, 0, SIZE_VEC3)
/**
* \~english @brief Clears a four dimensional vector.
* \~japanese @brief 四次元ベクトルをクリアする。
*/
#define clearVec4(vec) memset(vec, 0, SIZE_VEC4)

/**
* \~english @brief Copys a two dimensional vector.
* \~japanese @brief 二次元ベクトルをコピーする。
*/
#define copyVec2(dest, src) memcpy_s(dest, SIZE_VEC2, src, SIZE_VEC2)
/**
* \~english @brief Copys a three dimensional vector.
* \~japanese @brief 三次元ベクトルをコピーする。
*/
#define copyVec3(dest, src) memcpy_s(dest, SIZE_VEC3, src, SIZE_VEC3)
/**
* \~english @brief Copys a four dimensional vector.
* \~japanese @brief 四次元ベクトルをコピーする。
*/
#define copyVec4(dest, src) memcpy_s(dest, SIZE_VEC4, src, SIZE_VEC4)

/**
* \~english @brief Converts a three dimensional vector to a four dimensional vector.
* \~japanese @brief 三次元ベクトルを四次元ベクトルへ変換する。
*/
float *convVec3toVec4(const float in[3], float out[4]);
/**
* \~english @brief Converts a four dimensional vector to a three dimensional vector.
* \~japanese @brief 四次元ベクトルを三次元ベクトルへ変換する。
*/
float *convVec4toVec3(const float in[4], float out[3]);
/**
* \~english @brief Extracts components from a three dimensional vector.
* \~japanese @brief 三次元ベクトルから要素を抽出する。
*/
float *extractComponents3(const float in[3], int mask, float out[3]);

/**
* \~english @brief Converts a four squared dimensional matrix to a three squared dimensional matrix.
* \~japanese @brief 四次元正方行列を三次元正方行列へ変換する。
*/
float (*convMat4toMat3(float in[4][4], float out[3][3]))[3];

/**
* \~english @brief Initializes a 3D vector. Components specified by the mask are set to 1.
* \~japanese @brief 三次元ベクトルを初期化します。マスクで指定された要素は１にセットされます。
*/
float* initVec3(float vec[3], int mask);
/**
* \~english @brief Set a value for components specified by the mask.
* \~japanese @brief マスクで指定された要素に値をセットします。
*/
float* setVec3(float vec[3], float val, int mask);
/**
* \~english @brief Checks wheather two 3d vectors are equal or not.
* \~japanese @brief 二つの三次元ベクトルが等しいかどうかをチェックします。
*/
int equalVec3(float a[3], float b[3]);
/**
* \~english @brief Returns the index of 3d vector that has the maximum absolute value.
* \~japanese @brief 三次元ベクトルの絶対値が最大なインデックスを返します。
*/
int maxAbsIndex3(float vector[3]);
/**
* \~english @brief Returns cos() of two 3d vectors.
* \~japanese @brief ２つの三次元ベクトルのコサインを返します。
*/
float cosVec3(float a[3], float b[3]);

/**
* \~english @brief Returns the dot of two 2d vectors.
* \~japanese @brief 二次元ベクトルの内積を返します。
*/
float dot2(const float a[2], const float b[2]);
/**
* \~english @brief Returns the dot of two 3d vectors.
* \~japanese @brief ３次元ベクトルの内積を返します。
*/
float dot3(const float a[3], const float b[3]);
/**
* \~english @brief Returns the dot of two 4d vectors.
* \~japanese @brief ４次元ベクトルの内積を返します。
*/
float dot4(const float a[4], const float b[4]);
/**
* \~english @brief Returns the cross of two 3d vectors.
* \~japanese @brief ３次元ベクトルの外積を返します。
*/
float* cross(const float a[3], const float b[3], float out[3]);
/**
* \~english @brief Returns the area of the triangle.
* \~japanese @brief 三角形の面積を返します。
*/
float areaOfTriangle(float triangle[3][3]);
/**
* \~english @brief Returns the magnitude of the 2d vector.
* \~japanese @brief 二次元ベクトルの大きさを返します。
*/
float length2(const float vector[2]);
/**
* \~english @brief Returns the magnitude of the 3d vector.
* \~japanese @brief ３次元ベクトルの大きさを返します。
*/
float length3(const float vector[3]);
/**
* \~english @brief Returns the distance between two 2d coordinates.
* \~japanese @brief 二点の二次元座標の距離を返します。
*/
float distance2(const float a[2], const float b[2]);
/**
* \~english @brief Returns the distance between two 3d coordinates.
* \~japanese @brief 二点の３次元座標の距離を返します。
*/
float distance3(const float a[3], const float b[3]);
/**
* \~english @brief Add two 2d vectors.
* \~japanese @brief 二つの二次元ベクトルを足します。
*/
float* addVec2(const float a[2], const float b[2], float out[2]);
/**
* \~english @brief Add two 3d vectors.
* \~japanese @brief 二つの３次元ベクトルを足します。
*/
float* addVec3(const float a[3], const float b[3], float out[3]);
/**
* \~english @brief Substracts a 2d vector from another 2d vector.
* \~japanese @brief ある二次元ベクトルからほかの二次元ベクトルを引きます。
*/
float* subVec2(const float a[2], const float b[2], float out[2]);
/**
* \~english @brief Substracts a 3d vector from another 3d vector.
* \~japanese @brief ある３次元ベクトルからほかの３次元ベクトルを引きます。
*/
float* subVec3(const float a[3], const float b[3], float out[3]);
/**
* \~english @brief Multiples a 2d vector by a scalar.
* \~japanese @brief 二次元ベクトルをスカラー倍します。
*/
float* mulVec2ByScalar(const float vector[2], float scalar, float out[2]);
/**
* \~english @brief Divides a 2d vector by a scalar.
* \~japanese @brief 二次元ベクトルをスカラーで割ります。
*/
float* divVec2ByScalar(const float vector[2], float scalar, float out[2]);
/**
* \~english @brief Multiples a 3d vector by a scalar.
* \~japanese @brief ３次元ベクトルをスカラー倍します。
*/
float* mulVec3ByScalar(const float vector[3], float scalar, float out[3]);
/**
* \~english @brief Divides a 3d vector by a scalar.
* \~japanese @brief ３次元ベクトルをスカラーで割ります。
*/
float* divVec3ByScalar(const float vector[3], float scalar, float out[3]);
/**
* \~english @brief Multiples a 4d vector by a scalar.
* \~japanese @brief ４次元ベクトルをスカラー倍します。
*/
float* mulVec4ByScalar(const float vector[4], float scalar, float out[4]);
/**
* \~english @brief Divides a 4d vector by a scalar.
* \~japanese @brief ４次元ベクトルをスカラーで割ります。
*/
float* divVec4ByScalar(const float vector[4], float scalar, float out[4]);
/**
* \~english @brief Normalize a 2d vector.
* \~japanese @brief 二次元ベクトルを正規化します。
*/
float* normalize2(const float vector[2], float out[2]);
/**
* \~english @brief Normalize a 3d vector.
* \~japanese @brief ３次元ベクトルを正規化します。
*/
float* normalize3(const float vector[3], float out[3]);
/**
* \~english @brief Calculates the angle between two 2d vectors.
* \~japanese @brief ２つの二次元ベクトル間の角度を求めます。
*/
float* direction2(const float a[2], const float b[2], float out[2]);
/**
* \~english @brief Calculates the angle between two 3d vectors.
* \~japanese @brief ２つの３次元ベクトル間の角度を求めます。
*/
float* direction3(const float a[3], const float b[3], float out[3]);
/**
* \~english @brief Calculates the angle of the 2d vector.
* \~japanese @brief 二次元ベクトルの角度を求めます。
*/
float angleVec2(const float vector[2]);

/**
* \~english @brief Adds two 3d squared matrices.
* \~japanese @brief ２つの三次元正方行列を足します。
*/
float (*addMat3(const float a[3][3], const float b[3][3], float out[3][3]))[3];
/**
* \~english @brief Multiplies two 3d squared matrices.
* \~japanese @brief ２つの三次元正方行列を掛けます。
*/
float	(*mulMat3(const float a[3][3], const float b[3][3], float out[3][3]))[3];
/**
* \~english @brief Multiplies two 4d squared matrices.
* \~japanese @brief ２つの４次元正方行列を掛けます。
*/
float	(*mulMat4(const float a[4][4], const float b[4][4], float out[4][4]))[4];
/**
* \~english @brief Multiplies a 3d squared matrix by a scalar.
* \~japanese @brief 三次元正方行列にスカラーを掛けます。
*/
float	(*mulMat3ByScalar(const float in[3][3], float scalar, float out[3][3]))[3];
/**
* \~english @brief Divdes a 3d squared matrix by a scalar.
* \~japanese @brief 三次元正方行列をスカラーで割ります。
*/
float	(*divMat3ByScalar(const float in[3][3], float scalar, float out[3][3]))[3];
/**
* \~english @brief Divdes a 4d squared matrix by a scalar.
* \~japanese @brief ４次元正方行列をスカラーで割ります。
*/
float	(*divMat4ByScalar(const float in[4][4], float scalar, float out[4][4]))[4];
/**
* \~english @brief Multiplies a 3d squared matrix by a 3d vector.
* \~japanese @brief 三次元正方行列に3次元ベクトルを掛けます。
*/
float* mulMat3Vec3(const float mat[3][3], const float vec[3], float out[3]);
/**
* \~english @brief Multiplies a 4d squared matrix by a 4d vector.
* \~japanese @brief ４次元正方行列に４次元ベクトルを掛けます。
*/
float* mulMat4Vec4(const float mat[4][4], const float vec[4], float out[4]);
/**
* \~english @brief Multiplies a 4d squared matrix by a 4d vector. (For projection usage.)
* \~japanese @brief ４次元正方行列に４次元ベクトルを掛けます。（投影用）
*/
float* mulMat4Vec4Proj(const float mat[4][4], const float vec[4], float out[4]);
/**
* \~english @brief Transposes a 3d squared matrix.
* \~japanese @brief ３次元正方行列を転置します。
*/
float (*transposeMat3(const float mat[3][3], float out[3][3]))[3];
/**
* \~english @brief Transposes a 4d squared matrix.
* \~japanese @brief ４次元正方行列を転置します。
*/
float (*transposeMat4(const float mat[4][4], float out[4][4]))[4];

/**
* \~english @brief Calculates a cofactor of a 3d squared matrix.
* \~japanese @brief ３次元正方行列の余因子を求めます。
*/
float (*cofactor3(const float in[3][3], int row, int col, float out[2][2]))[2];
/**
* \~english @brief Calculates a cofactor of a 4d squared matrix.
* \~japanese @brief ４次元正方行列の余因子を求めます。
*/
float (*cofactor4(const float in[4][4], int row, int col, float out[3][3]))[3];
/**
* \~english @brief Calculates a cofactor matrix of a 3d squared matrix.
* \~japanese @brief ３次元正方行列の余因子の行列を求めます。
*/
float (*cofactorsMat3(const float in[3][3], float out[3][3]))[3];
/**
* \~english @brief Calculates a cofactor matrix of a 4d squared matrix.
* \~japanese @brief ４次元正方行列の余因子の行列を求めます。
*/
float (*cofactorsMat4(const float in[4][4], float out[4][4]))[4];
/**
* \~english @brief Calculates an adjugate matrix of a 3d squared matrix.
* \~japanese @brief ３次元正方行列の余因子行列を求めます。
*/
float (*adjugate3(float in[3][3], float out[3][3]))[3];
/**
* \~english @brief Calculates an adjugate matrix of a 4d squared matrix.
* \~japanese @brief ４次元正方行列の余因子行列を求めます。
*/
float (*adjugate4(float in[4][4], float out[4][4]))[4];
/**
* \~english @brief Calculates the determinant of the 2d squared matrix.
* \~japanese @brief ２次元正方行列の行列式の値を求めます。
*/
float det2(float in[2][2]);
/**
* \~english @brief Calculates the determinant of the 3d squared matrix.
* \~japanese @brief ３次元正方行列の行列式の値を求めます。
*/
float det3(float in[3][3]);
/**
* \~english @brief Calculates the determinant of the 4d squared matrix.
* \~japanese @brief ４次元正方行列の行列式の値を求めます。
*/
float det4(float in[4][4]);
/**
* \~english @brief Calculates the inverse of the 3d squared matrix.
* \~japanese @brief ３次元正方行列の逆行列を求めます。
*/
float (*inverse3(float in[3][3], float out[3][3]))[3];
/**
* \~english @brief Calculates the inverse of the 4d squared matrix.
* \~japanese @brief ４次元正方行列の逆行列を求めます。
*/
float (*inverse4(float in[4][4], float out[4][4]))[4];
/**
* \~english @brief Orthogonalize a 3d squared matrix.
* \~japanese @brief ３次元正方行列を直行化します。
*/
float (*orthogonalize3(float in[3][3], float out[3][3]))[3];

/**
* \~english @brief Generates the skew of the 3d squared matrix.
* \~japanese @brief ３次元正方行列のスキューを生成します。
*/
float (*genSkewMat3(float in[3], float out[3][3]))[3];
/**
* \~english @brief Generates a translate matrix in a 3d squared matrix.
* \~japanese @brief 移動を３次元正方行列で生成します。
*/
float (*genTranslationMat3(float dx, float dy, float mat[3][3]))[3];
/**
* \~english @brief Generates a translate matrix in a 4d squared matrix.
* \~japanese @brief 移動を４次元正方行列で生成します。
*/
float (*genTranslationMat4(float dx, float dy, float dz, float mat[4][4]))[4];
/**
* \~english @brief Generates a 3d squared identiry matrix.
* \~japanese @brief ３次元正方単位行列を生成します。
*/
float (*genIdentityMat3(float mat[3][3]))[3];
/**
* \~english @brief Generates a 4d squared identiry matrix.
* \~japanese @brief ４次元正方単位行列を生成します。
*/
float (*genIdentityMat4(float mat[4][4]))[4];
/**
* \~english @brief Generates a 3d squared scale matrix.
* \~japanese @brief スケールの３次元正方行列を生成します。
*/
float (*genScaleMat3(float sx, float sy, float mat[3][3]))[3];
/**
* \~english @brief Generates a 4d squared scale matrix.
* \~japanese @brief スケールの４次元正方行列を生成します。
*/
float (*genScaleMat4(float sx, float sy, float sz, float mat[4][4]))[4];
/**
* \~english @brief Generates a 3d squared rotation matrix.
* \~japanese @brief 回転の３次元正方行列を生成します。
*/
float (*genRotationMat3(float rotation, float mat[3][3]))[3];
/**
* \~english @brief Generates a 4d squared rotation matrix with respect to x axis.
* \~japanese @brief x軸についての回転の４次元正方行列を生成します。
*/
float (*genRotationXMat4(float rotation, float mat[4][4]))[4];
/**
* \~english @brief Generates a 4d squared rotation matrix with respect to y axis.
* \~japanese @brief ｙ軸についての回転の４次元正方行列を生成します。
*/
float (*genRotationYMat4(float rotation, float mat[4][4]))[4];
/**
* \~english @brief Generates a 4d squared rotation matrix with respect to z axis.
* \~japanese @brief ｚ軸についての回転の４次元正方行列を生成します。
*/
float (*genRotationZMat4(float rotation, float mat[4][4]))[4];
/**
* \~english @brief Generates a 4d squared rotation matrix.
* \~japanese @brief 回転の４次元正方行列を生成します。
*/
float (*genRotationMat4(float rx, float ry, float rz, float mat[4][4]))[4];

/**
* \~english @brief Calculates the angles of the 3d squared matrix.
* \~japanese @brief 三次元正方行列の角度を取得します。
*/
float *getAngleFromMat3(float in[3][3], float out[3]);

/**
* \~english @brief Generates a look-at matrix.
* \~japanese @brief ルックアト行列を生成します。
*/
float (*genLookAtMat4(float position[3], float target[3], float worldUp[3], float mat[4][4]))[4];
/**
* \~english @brief Generates a perspective matrix.
* \~japanese @brief 投影行列を生成します。
*/
float (*genPerspectiveMat4(float fovY, float zNear, float zFar, float aspect, float mat[4][4]))[4];

/**
* \~english @brief Generates a inertia matrix for box.
* \~japanese @brief 箱の慣性モーメントを生成します。
*/
float (*genInertiaTensorBox(float mass, float width, float height, float depth, float out[3][3]))[3];

/**
* \~english @brief Prints a 3d squared matrix to stdout.
* \~japanese @brief 三次元正方行列を標準出力に出力します。
*/
void printMat3(float mat[3][3]);
/**
* \~english @brief Prints a 4d squared matrix to stdout.
* \~japanese @brief ４次元正方行列を標準出力に出力します。
*/
void printMat4(float mat[4][4]);

#endif
