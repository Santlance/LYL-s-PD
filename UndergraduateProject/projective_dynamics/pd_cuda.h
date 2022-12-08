#pragma once

#include <cuda.h>
#include <cuda_runtime.h>
//#include "cuda_gl_interop.h"
#include "device_functions.h"
#include "../header.h"
#include <cstdio>

#define CUDA_THREAD_NUMBER 512


extern int vertex_number_cuda;				// ������Ŀ
extern int tet_number_cuda;				// ��������Ŀ

extern float* positions_cuda;				// λ��
extern float* last_positions_cuda;			// ��һ��λ��
extern float* old_positions_cuda;			// sn
extern float* prev_positions_cuda;			// ��һ�ε���
extern float* next_positions_cuda;			// ��һ��λ��
extern float* velocity_cuda;				// �ٶ�
extern float* external_force_cuda;			// ����
extern float* mass_cuda;					// ����

extern float* volumeDiag_cuda;				// �Խ�Ԫ��
extern float* tetInvD3x3_cuda;				// 
extern float* tetInvD3x4_cuda;
extern float* tetVolume_cuda;
extern int* tet_indices_cuda;				// ����������
extern unsigned int* tet_draw_indices_cuda;
extern float* tet_stiffness_cuda;			// �����嵯��ϵ��

extern float* force_cuda;					// ��������
extern float* fixed_cuda;					// �����Ƿ�̶�

// λ��Լ��
extern int pos_constraint_num_cuda;					// Լ������
extern unsigned int* pos_constraint_indices_cuda;	// ����
extern float* pos_constraint_targets_cuda;			// Ŀ��λ��

// surface
extern unsigned int face_number_cuda;
extern unsigned int* tet_faces_indices_cuda;		// ��������������
extern float* normals_cuda;						// ÿ������ķ���


// OpenGL
extern cudaGraphicsResource* tet_positions_gl;
extern cudaGraphicsResource* tet_indices_gl;
extern cudaGraphicsResource* tet_face_indices_gl;
extern cudaGraphicsResource* tet_normals_gl;



// ------------------------- ���ú����ӿ� --------------------------------------

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


