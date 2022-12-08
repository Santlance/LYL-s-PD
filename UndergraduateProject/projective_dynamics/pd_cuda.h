#pragma once

#include <cuda.h>
#include <cuda_runtime.h>
//#include "cuda_gl_interop.h"
#include "device_functions.h"
#include "../header.h"
#include <cstdio>

#define CUDA_THREAD_NUMBER 512


extern int vertex_number_cuda;				// 顶点数目
extern int tet_number_cuda;				// 四面体数目

extern float* positions_cuda;				// 位置
extern float* last_positions_cuda;			// 上一个位置
extern float* old_positions_cuda;			// sn
extern float* prev_positions_cuda;			// 上一次迭代
extern float* next_positions_cuda;			// 下一个位置
extern float* velocity_cuda;				// 速度
extern float* external_force_cuda;			// 外力
extern float* mass_cuda;					// 质量

extern float* volumeDiag_cuda;				// 对角元素
extern float* tetInvD3x3_cuda;				// 
extern float* tetInvD3x4_cuda;
extern float* tetVolume_cuda;
extern int* tet_indices_cuda;				// 四面体索引
extern unsigned int* tet_draw_indices_cuda;
extern float* tet_stiffness_cuda;			// 四面体弹性系数

extern float* force_cuda;					// 顶点受力
extern float* fixed_cuda;					// 顶点是否固定

// 位置约束
extern int pos_constraint_num_cuda;					// 约束个数
extern unsigned int* pos_constraint_indices_cuda;	// 索引
extern float* pos_constraint_targets_cuda;			// 目标位置

// surface
extern unsigned int face_number_cuda;
extern unsigned int* tet_faces_indices_cuda;		// 表面三角形索引
extern float* normals_cuda;						// 每个顶点的法线


// OpenGL
extern cudaGraphicsResource* tet_positions_gl;
extern cudaGraphicsResource* tet_indices_gl;
extern cudaGraphicsResource* tet_face_indices_gl;
extern cudaGraphicsResource* tet_normals_gl;



// ------------------------- 调用函数接口 --------------------------------------

extern "C"
void cuda_calculate_Sn(float damping, float dt);

extern "C"
void cuda_calculate_tet();

extern "C"
void cuda_calculate_positions(float omega, float dt);

extern "C"
void cuda_calculate_velocity(float dt);

extern "C"
void cuda_calculate_position_constraint_force(int stiffness);

extern "C"
void cuda_update_normals();

// ------------------------------------------------------------------------------


