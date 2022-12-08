#include "pd_cuda.h"

#define GRAVITY -10.0
//#define GRAVITY 0

int vertex_number_cuda;				// 顶点数目
int tet_number_cuda;				// 四面体数目

float* positions_cuda;				// 位置
float* last_positions_cuda;			// 上一个位置（上一帧的位置）
float* old_positions_cuda;			// sn
float* prev_positions_cuda;			// 上一次迭代的位置
float* next_positions_cuda;			// 下一个位置
float* velocity_cuda;				// 速度
float* external_force_cuda;			// 外力
float* mass_cuda;					// 质量

float* volumeDiag_cuda;				// 对角元素
float* tetInvD3x3_cuda;				// 
float* tetInvD3x4_cuda;
float* tetVolume_cuda;
int* tet_indices_cuda;				// 四面体索引
unsigned int* tet_draw_indices_cuda;
float* tet_stiffness_cuda;			// 四面体弹性系数

float* force_cuda;					// 顶点受力
float* fixed_cuda;					// 顶点是否固定

// 位置约束
int pos_constraint_num_cuda;				// 约束个数
unsigned int* pos_constraint_indices_cuda;	// 索引
float* pos_constraint_targets_cuda;			// 目标位置

// surface
unsigned int face_number_cuda;
unsigned int* tet_faces_indices_cuda;		// 表面三角形索引
float* normals_cuda;						// 每个顶点的法线


// OpenGL
cudaGraphicsResource* tet_positions_gl;
cudaGraphicsResource* tet_indices_gl;
cudaGraphicsResource* tet_face_indices_gl;
cudaGraphicsResource* tet_normals_gl;


/*
* 注意 EigenMatrixX.data() 跟矩阵的对应关系是一列一列的，也就是说
*	EigenMatrixX.data() = [1, 2, 3, 4, 5, 6, 7, 8, 9]
* 对应的矩阵为
*		[1, 4, 7]
*		[2, 5, 8]
*		[3, 6, 9]
*/


// ------------------------------------------------------------------------------
// ------------------------- CUDA 核函数 ----------------------------------------
// ------------------------------------------------------------------------------

// *************************** device ******************************

// R = A * B
__device__ void matrix_product_33(float* R, float* A, float* B)
{
	R[0] = A[0] * B[0] + A[1] * B[3] + A[2] * B[6];
	R[1] = A[0] * B[1] + A[1] * B[4] + A[2] * B[7];
	R[2] = A[0] * B[2] + A[1] * B[5] + A[2] * B[8];
	R[3] = A[3] * B[0] + A[4] * B[3] + A[5] * B[6];
	R[4] = A[3] * B[1] + A[4] * B[4] + A[5] * B[7];
	R[5] = A[3] * B[2] + A[4] * B[5] + A[5] * B[8];
	R[6] = A[6] * B[0] + A[7] * B[3] + A[8] * B[6];
	R[7] = A[6] * B[1] + A[7] * B[4] + A[8] * B[7];
	R[8] = A[6] * B[2] + A[7] * B[5] + A[8] * B[8];
}

// R = A * B
__device__ void matrix_product(float* R, float* A, float* B, int m, int n, int p)
{
	memset(R, 0, sizeof(float) * m * p);
	for (int i = 0; i < m; i++)
		for (int j = 0; j < p; j++)
			for (int k = 0; k < n; k++)
				R[i * p + j] += A[i * n + k] * B[k * p + j];
}

// R = A - B
__device__ void matrix_substract_33(float* R, float* A, float* B)
{
	R[0] = A[0] - B[0];
	R[1] = A[1] - B[1];
	R[2] = A[2] - B[2];
	R[3] = A[3] - B[3];
	R[4] = A[4] - B[4];
	R[5] = A[5] - B[5];
	R[6] = A[6] - B[6];
	R[7] = A[7] - B[7];
	R[8] = A[8] - B[8];
}

// 从形变梯度F提取R
__device__ void get_R_from_F(float F[3][3], float R[3][3])
{
	float C[3][3];
	memset(&C[0][0], 0, sizeof(float) * 9);
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			for (int k = 0; k < 3; k++)
				C[i][j] += F[k][i] * F[k][j];

	float C2[3][3];
	memset(&C2[0][0], 0, sizeof(float) * 9);
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			for (int k = 0; k < 3; k++)
				C2[i][j] += C[i][k] * C[j][k];

	float det = F[0][0] * F[1][1] * F[2][2] +
		F[0][1] * F[1][2] * F[2][0] +
		F[1][0] * F[2][1] * F[0][2] -
		F[0][2] * F[1][1] * F[2][0] -
		F[0][1] * F[1][0] * F[2][2] -
		F[0][0] * F[1][2] * F[2][1];

	float I_c = C[0][0] + C[1][1] + C[2][2];
	float I_c2 = I_c * I_c;
	float II_c = 0.5 * (I_c2 - C2[0][0] - C2[1][1] - C2[2][2]);
	float III_c = det * det;
	float k = I_c2 - 3 * II_c;

	float inv_U[3][3];
	if (k < 1e-10f)
	{
		float inv_lambda = 1 / sqrt(I_c / 3);
		memset(inv_U, 0, sizeof(float) * 9);
		inv_U[0][0] = inv_lambda;
		inv_U[1][1] = inv_lambda;
		inv_U[2][2] = inv_lambda;
	}
	else
	{
		float l = I_c * (I_c * I_c - 4.5 * II_c) + 13.5 * III_c;
		float k_root = sqrt(k);
		float value = l / (k * k_root);
		if (value < -1.0) value = -1.0;
		if (value > 1.0) value = 1.0;
		float phi = acos(value);
		float lambda2 = (I_c + 2 * k_root * cos(phi / 3)) / 3.0;
		float lambda = sqrt(lambda2);

		float III_u = sqrt(III_c);
		if (det < 0)   III_u = -III_u;
		float I_u = lambda + sqrt(-lambda2 + I_c + 2 * III_u / lambda);
		float II_u = (I_u * I_u - I_c) * 0.5;

		float U[3][3];
		float inv_rate, factor;

		inv_rate = 1 / (I_u * II_u - III_u);
		factor = I_u * III_u * inv_rate;

		memset(U, 0, sizeof(float) * 9);
		U[0][0] = factor;
		U[1][1] = factor;
		U[2][2] = factor;

		factor = (I_u * I_u - II_u) * inv_rate;
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				U[i][j] += factor * C[i][j] - inv_rate * C2[i][j];

		inv_rate = 1 / III_u;
		factor = II_u * inv_rate;
		memset(inv_U, 0, sizeof(float) * 9);
		inv_U[0][0] = factor;
		inv_U[1][1] = factor;
		inv_U[2][2] = factor;


		factor = -I_u * inv_rate;
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				inv_U[i][j] += factor * U[i][j] + inv_rate * C[i][j];
	}

	memset(&R[0][0], 0, sizeof(float) * 9);
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			for (int k = 0; k < 3; k++)
				R[i][j] += F[i][k] * inv_U[k][j];

	//检查，避免invert
	if (det <= 0)
	{
		R[0][0] = 1;
		R[0][1] = 0;
		R[0][2] = 0;
		R[1][0] = 0;
		R[1][1] = 1;
		R[1][2] = 0;
		R[2][0] = 0;
		R[2][1] = 0;
		R[2][2] = 1;
	}
}

// ***********************************************************



// *************************** global ******************************

__global__ void calculate_Sn(float* positions, float* velocity, float* external_force,
	float* old_positions, float* prev_positions, float* last_positions, float* fixed,
	int vertices_number, float damping, float dt)
{
	unsigned int threadId = blockIdx.x * blockDim.x + threadIdx.x;

	if (threadId >= vertices_number) return;

	int idx = threadId * 3 + 0;
	int idy = threadId * 3 + 1;
	int idz = threadId * 3 + 2;

	last_positions[idx] = positions[idx];
	last_positions[idy] = positions[idy];
	last_positions[idz] = positions[idz];

	// 固定点不运动
	if (fixed[threadId] < 10.0)
	{
		// 运动阻尼
		velocity[idx] *= damping;
		velocity[idy] *= damping;
		velocity[idz] *= damping;

		// 添加重力
		velocity[idy] += GRAVITY * dt;

		// 其他外力
		velocity[idx] += external_force[idx] * dt;
		velocity[idy] += external_force[idy] * dt;
		velocity[idz] += external_force[idz] * dt;

		// 位置预测
		positions[idx] += velocity[idx] * dt;
		positions[idy] += velocity[idy] * dt;
		positions[idz] += velocity[idz] * dt;
	}

	// old_positions 记录 Sn
	old_positions[idx] = positions[idx];
	old_positions[idy] = positions[idy];
	old_positions[idz] = positions[idz];
	prev_positions[idx] = positions[idx];
	prev_positions[idy] = positions[idy];
	prev_positions[idz] = positions[idz];

	// 外力清零，外力应当每时刻更新
	external_force[idx] = 0;
	external_force[idy] = 0;
	external_force[idz] = 0;
}

__global__ void calculate_tet(float* positions, int* tetIndex, float* tetInvD3x3,
	float* tetInvD3x4, float* force, float* tet_volume, int tet_number, float* tet_stiffness)
{
	unsigned int threadId = blockIdx.x * blockDim.x + threadIdx.x;

	if (threadId >= tet_number) return;

	// TODO active[]

	int id0 = tetIndex[threadId * 4 + 0];
	int id1 = tetIndex[threadId * 4 + 1];
	int id2 = tetIndex[threadId * 4 + 2];
	int id3 = tetIndex[threadId * 4 + 3];

	// shape 矩阵
	float D[9];

	D[0] = positions[id1 * 3 + 0] - positions[id0 * 3 + 0];
	D[1] = positions[id2 * 3 + 0] - positions[id0 * 3 + 0];
	D[2] = positions[id3 * 3 + 0] - positions[id0 * 3 + 0];
	D[3] = positions[id1 * 3 + 1] - positions[id0 * 3 + 1];
	D[4] = positions[id2 * 3 + 1] - positions[id0 * 3 + 1];
	D[5] = positions[id3 * 3 + 1] - positions[id0 * 3 + 1];
	D[6] = positions[id1 * 3 + 2] - positions[id0 * 3 + 2];
	D[7] = positions[id2 * 3 + 2] - positions[id0 * 3 + 2];
	D[8] = positions[id3 * 3 + 2] - positions[id0 * 3 + 2];

	// 计算形变梯度F
	float F[9];
	matrix_product_33(F, D, &tetInvD3x3[threadId * 9]);

	// 从 F 中分解出 R（形变梯度分解出旋转成分）
	float R[9];
	get_R_from_F((float(*)[3])F, (float(*)[3])R);

	// R = R - F
	matrix_substract_33(R, R, F);

	// force = - w * Ac^T * (F - R)	对能量求导可得
	float temp[12];
	matrix_product(temp, R, &tetInvD3x4[threadId * 12], 3, 3, 4);
	float coef = tet_volume[threadId] * tet_stiffness[threadId];
	// 原子操作，共享变量互斥操作
	atomicAdd(force + id0 * 3 + 0, temp[0] * coef);
	atomicAdd(force + id0 * 3 + 1, temp[4] * coef);
	atomicAdd(force + id0 * 3 + 2, temp[8] * coef);
	atomicAdd(force + id1 * 3 + 0, temp[1] * coef);
	atomicAdd(force + id1 * 3 + 1, temp[5] * coef);
	atomicAdd(force + id1 * 3 + 2, temp[9] * coef);
	atomicAdd(force + id2 * 3 + 0, temp[2] * coef);
	atomicAdd(force + id2 * 3 + 1, temp[6] * coef);
	atomicAdd(force + id2 * 3 + 2, temp[10] * coef);
	atomicAdd(force + id3 * 3 + 0, temp[3] * coef);
	atomicAdd(force + id3 * 3 + 1, temp[7] * coef);
	atomicAdd(force + id3 * 3 + 2, temp[11] * coef);
}

__global__ void calculate_position(float* positions, float* force, float* mass,
	float* next_positions, float* prev_positions, float* old_positions, float* volumeDiag, float* fixed,
	int vertices_number, float dt, float omega)
{
	unsigned int threadId = blockIdx.x * blockDim.x + threadIdx.x;

	if (threadId >= vertices_number) return;

	//

	int idx = threadId * 3 + 0;
	int idy = threadId * 3 + 1;
	int idz = threadId * 3 + 2;

	// M / (h^2)
	float diagConstant = (mass[threadId] + fixed[threadId]) / (dt * dt);

	// shape match?

	// x_n1 = D^-1 * (M / h^2 * (Sn - x_n) + Sigma(w * A^T *(R - F))) + x_n  按照分量计算
	next_positions[idx] = (diagConstant * (old_positions[idx] - positions[idx]) + force[idx]) / (volumeDiag[threadId] + diagConstant) + positions[idx];
	next_positions[idy] = (diagConstant * (old_positions[idy] - positions[idy]) + force[idy]) / (volumeDiag[threadId] + diagConstant) + positions[idy];
	next_positions[idz] = (diagConstant * (old_positions[idz] - positions[idz]) + force[idz]) / (volumeDiag[threadId] + diagConstant) + positions[idz];

	// 内力清零
	force[idx] = force[idy] = force[idz] = 0;

	// 切比雪夫迭代
	next_positions[idx] = (next_positions[idx] - positions[idx]) * 0.6 + positions[idx];
	next_positions[idy] = (next_positions[idy] - positions[idy]) * 0.6 + positions[idy];
	next_positions[idz] = (next_positions[idz] - positions[idz]) * 0.6 + positions[idz];

	next_positions[idx] = omega * (next_positions[idx] - prev_positions[idx]) + prev_positions[idx];
	next_positions[idy] = omega * (next_positions[idy] - prev_positions[idy]) + prev_positions[idy];
	next_positions[idz] = omega * (next_positions[idz] - prev_positions[idz]) + prev_positions[idz];

	// 更新
	prev_positions[idx] = positions[idx];
	prev_positions[idy] = positions[idy];
	prev_positions[idz] = positions[idz];

	positions[idx] = next_positions[idx];
	positions[idy] = next_positions[idy];
	positions[idz] = next_positions[idz];
}

__global__ void calculate_velocity(float* positions, float* velocity,
	float* last_positions, int vertices_number, float dt)
{
	unsigned int threadId = blockIdx.x * blockDim.x + threadIdx.x;

	if (threadId >= vertices_number) return;

	int idx = threadId * 3 + 0;
	int idy = threadId * 3 + 1;
	int idz = threadId * 3 + 2;

	// 更新速度
	velocity[idx] = (positions[idx] - last_positions[idx]) / dt;
	velocity[idy] = (positions[idy] - last_positions[idy]) / dt;
	velocity[idz] = (positions[idz] - last_positions[idz]) / dt;

	// 这里临时这么写，在下面模拟一个平面
	//float ground = -10;
	float ground = -0.402;
	if (positions[idy] < ground)
	{
		positions[idy] = ground;

		velocity[idx] = 0;
		velocity[idy] = -velocity[idy];
		velocity[idz] = 0;
	}
}

__global__ void calculate_position_constraint_force(float* position, float* force, int constraint_number,
	unsigned int* constraint_indices, float* constraint_target, int stiffness)
{
	unsigned int threadId = blockIdx.x * blockDim.x + threadIdx.x;

	if (threadId >= constraint_number) return;

	// 约束点的id
	unsigned int id = constraint_indices[threadId];

	// 约束点坐标
	float px = position[id * 3 + 0];
	float py = position[id * 3 + 1];
	float pz = position[id * 3 + 2];
	// 目标点坐标
	float tx = constraint_target[threadId * 3 + 0];
	float ty = constraint_target[threadId * 3 + 1];
	float tz = constraint_target[threadId * 3 + 2];

	// 约束点到目标点的运动方向
	float dx = tx - px;
	float dy = ty - py;
	float dz = tz - pz;

	// 以距离为系数添加受力
	atomicAdd(force + id * 3 + 0, dx * stiffness);
	atomicAdd(force + id * 3 + 1, dy * stiffness);
	atomicAdd(force + id * 3 + 2, dz * stiffness);
}


__global__ void clear_normal(float* normal, int vertex_number)
{
	unsigned int threadId = blockIdx.x * blockDim.x + threadIdx.x;

	if (threadId >= vertex_number) return;

	normal[threadId * 3 + 0] = 0;
	normal[threadId * 3 + 1] = 0;
	normal[threadId * 3 + 2] = 0;
}

__global__ void update_normal(float* position, float* normal, unsigned int* face_indices, int face_num)
{
	unsigned int threadId = blockIdx.x * blockDim.x + threadIdx.x;

	if (threadId >= face_num) return;

	unsigned int id0 = face_indices[threadId * 3 + 0];
	unsigned int id1 = face_indices[threadId * 3 + 1];
	unsigned int id2 = face_indices[threadId * 3 + 2];

	// 三角形两条边向量
	float ax = position[id1 * 3 + 0] - position[id0 * 3 + 0];
	float ay = position[id1 * 3 + 1] - position[id0 * 3 + 1];
	float az = position[id1 * 3 + 2] - position[id0 * 3 + 2];
	float bx = position[id2 * 3 + 0] - position[id0 * 3 + 0];
	float by = position[id2 * 3 + 1] - position[id0 * 3 + 1];
	float bz = position[id2 * 3 + 2] - position[id0 * 3 + 2];

	// 叉乘计算法线
	float crossx = ay * bz - by * az;
	float crossy = az * bx - bz * ax;
	float crossz = ax * by - bx * ay;

	// 法线单位化
	float len = sqrt(crossx * crossx + crossy * crossy + crossz * crossz);
	crossx /= len;
	crossy /= len;
	crossz /= len;

	// 法线累加到顶点上
	atomicAdd(normal + id0 * 3 + 0, crossx);
	atomicAdd(normal + id0 * 3 + 1, crossy);
	atomicAdd(normal + id0 * 3 + 2, crossz);
	atomicAdd(normal + id1 * 3 + 0, crossx);
	atomicAdd(normal + id1 * 3 + 1, crossy);
	atomicAdd(normal + id1 * 3 + 2, crossz);
	atomicAdd(normal + id2 * 3 + 0, crossx);
	atomicAdd(normal + id2 * 3 + 1, crossy);
	atomicAdd(normal + id2 * 3 + 2, crossz);
}

__global__ void normalize_normal(float* normal, int vertex_number)
{
	unsigned int threadId = blockIdx.x * blockDim.x + threadIdx.x;

	if (threadId >= vertex_number) return;

	float nx = normal[threadId * 3 + 0];
	float ny = normal[threadId * 3 + 1];
	float nz = normal[threadId * 3 + 2];

	float len = sqrt(nx * nx + ny * ny + nz * nz);
	normal[threadId * 3 + 0] = nx / len;
	normal[threadId * 3 + 1] = ny / len;
	normal[threadId * 3 + 2] = nz / len;
}

// ***********************************************************

// ------------------------------------------------------------------------------
// ------------------------------------------------------------------------------
// ------------------------------------------------------------------------------




void cuda_calculate_Sn(float damping, float dt)
{
	// 每次update会首先调用这个函数，因此需要绑定CUDA和OpenGL的共享区
	cudaGraphicsMapResources(1, &tet_positions_gl, 0);
	size_t size = vertex_number_cuda * 3 * sizeof(float);
	cudaGraphicsResourceGetMappedPointer((void**)&positions_cuda, &size, tet_positions_gl);		// 获取内存指针

	cudaGraphicsMapResources(1, &tet_indices_gl, 0);
	size = tet_number_cuda * 12 * sizeof(unsigned int);
	cudaGraphicsResourceGetMappedPointer((void**)&tet_draw_indices_cuda, &size, tet_indices_gl);

	// 解算
	int block_number = (vertex_number_cuda + CUDA_THREAD_NUMBER - 1) / CUDA_THREAD_NUMBER;

	calculate_Sn << <block_number, CUDA_THREAD_NUMBER >> > (
		positions_cuda, velocity_cuda, external_force_cuda, old_positions_cuda,
		prev_positions_cuda, last_positions_cuda, fixed_cuda, vertex_number_cuda, damping, dt);

	// 同步
	cudaDeviceSynchronize();
}

void cuda_calculate_tet()
{
	int block_number = (tet_number_cuda + CUDA_THREAD_NUMBER - 1) / CUDA_THREAD_NUMBER;

	calculate_tet << < block_number, CUDA_THREAD_NUMBER >> > (
		positions_cuda, tet_indices_cuda, tetInvD3x3_cuda, tetInvD3x4_cuda,
		force_cuda, tetVolume_cuda, tet_number_cuda, tet_stiffness_cuda);

	cudaDeviceSynchronize();
}

void cuda_calculate_positions(float omega, float dt)
{
	int block_number = (vertex_number_cuda + CUDA_THREAD_NUMBER - 1) / CUDA_THREAD_NUMBER;

	calculate_position << <block_number, CUDA_THREAD_NUMBER >> > (
		positions_cuda, force_cuda, mass_cuda, next_positions_cuda, prev_positions_cuda,
		old_positions_cuda, volumeDiag_cuda, fixed_cuda, vertex_number_cuda, dt, omega
		);

	cudaDeviceSynchronize();
}

void cuda_calculate_velocity(float dt)
{
	int block_number = (vertex_number_cuda + CUDA_THREAD_NUMBER - 1) / CUDA_THREAD_NUMBER;

	calculate_velocity << <block_number, CUDA_THREAD_NUMBER >> > (
		positions_cuda, velocity_cuda, last_positions_cuda, vertex_number_cuda, dt
		);

	// 这是PD的最后一步，要解除绑定，否则OpenGL无法渲染
	cudaGraphicsUnmapResources(1, &tet_indices_gl, 0);
	cudaGraphicsUnmapResources(1, &tet_positions_gl, 0);

	cudaDeviceSynchronize();
}

void cuda_calculate_position_constraint_force(int stiffness)
{
	int block_number = (pos_constraint_num_cuda + CUDA_THREAD_NUMBER - 1) / CUDA_THREAD_NUMBER;

	calculate_position_constraint_force << <block_number, CUDA_THREAD_NUMBER >> > (
		positions_cuda, force_cuda, pos_constraint_num_cuda,
		pos_constraint_indices_cuda, pos_constraint_targets_cuda, stiffness
		);

	cudaDeviceSynchronize();
}

void cuda_update_normals()
{
	// map
	cudaGraphicsMapResources(1, &tet_positions_gl, 0);
	size_t size = vertex_number_cuda * 3 * sizeof(float);
	cudaGraphicsResourceGetMappedPointer((void**)&positions_cuda, &size, tet_positions_gl);		// 获取内存指针

	cudaGraphicsMapResources(1, &tet_normals_gl, 0);
	cudaGraphicsResourceGetMappedPointer((void**)&normals_cuda, &size, tet_normals_gl);

	// update
	int block_number = (vertex_number_cuda + CUDA_THREAD_NUMBER - 1) / CUDA_THREAD_NUMBER;
	clear_normal << <block_number, CUDA_THREAD_NUMBER >> > (normals_cuda, vertex_number_cuda);

	block_number= (face_number_cuda + CUDA_THREAD_NUMBER - 1) / CUDA_THREAD_NUMBER;
	update_normal << <block_number, CUDA_THREAD_NUMBER >> > (positions_cuda, normals_cuda, tet_faces_indices_cuda, face_number_cuda);
	
	block_number = (vertex_number_cuda + CUDA_THREAD_NUMBER - 1) / CUDA_THREAD_NUMBER;
	normalize_normal << <block_number, CUDA_THREAD_NUMBER >> > (normals_cuda, vertex_number_cuda);

	// unmap
	cudaGraphicsUnmapResources(1, &tet_indices_gl, 0);
	cudaGraphicsUnmapResources(1, &tet_normals_gl, 0);
	cudaDeviceSynchronize();
}
