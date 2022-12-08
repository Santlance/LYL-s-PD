#pragma once

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <iostream>

#include <cuda.h>
#include <cuda_runtime.h>
#include "cuda_gl_interop.h"
#include "device_functions.h"


#if defined(WIN32) || defined(_WIN32) || defined(WIN64)
#define FORCE_INLINE __forceinline
#else
#define FORCE_INLINE __attribute__((always_inline))
#endif


//
// ����d_float��ʹ���Ѿ�����
typedef float d_float;
// PBDֻ�漰CPU���㣬���¶���һ������PBD��ʹ��
typedef float Real;

typedef int d_int;
typedef unsigned int d_uint;

// eigen
typedef Eigen::Matrix<d_float, 4, 4> EigenMatrix4;
typedef Eigen::Matrix<d_float, 3, 3> EigenMatrix3;
typedef Eigen::Matrix<d_float, 2, 2> EigenMatrix2;
typedef Eigen::Matrix<d_float, 4, 1> EigenVector4;
typedef Eigen::Matrix<d_float, 3, 1> EigenVector3;
typedef Eigen::Matrix<d_float, 2, 1> EigenVector2;
typedef Eigen::Matrix<d_float, Eigen::Dynamic, 1> EigenVectorX;
typedef Eigen::Matrix<d_float, Eigen::Dynamic, Eigen::Dynamic> EigenMatrixX;
typedef Eigen::SparseMatrix<d_float> EigenSparseMatrix;
typedef Eigen::Triplet<d_float, d_int> EigenTriplet;
typedef Eigen::Quaternion<Real> Quaternionr;

using AlignedBox3r = Eigen::AlignedBox<Real, 3>;
using Vector3r = Eigen::Matrix<d_float, 3, 1>;
using Matrix3r = Eigen::Matrix<Real, 3, 3, Eigen::DontAlign>;

// eigen �д� EigenVectorX ȡ���� a ����ά������e.g. ������ϵͳ״̬q��ȡ������Ϊa�ĵ�����꣩
#define blockVector3(a) block<3, 1>(3 * (a), 0)

#define M_PI 3.14159265358979323846

// debug
#define PRINT(a) std::cout<< a << std::endl


// CAMERA
#define CAMERA_USER_MOVE

// OPENGL
#define DRAW_PHONG
//#define DRAW_LINES
//#define DRAW_SURFACE


