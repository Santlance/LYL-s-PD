#include "pd_simulation_cuda.h"

//
// 矩阵求逆
float Matrix_Inverse_3(float* A, float* R)				//R=inv(A)
{
	R[0] = A[4] * A[8] - A[7] * A[5];
	R[1] = A[7] * A[2] - A[1] * A[8];
	R[2] = A[1] * A[5] - A[4] * A[2];
	R[3] = A[5] * A[6] - A[3] * A[8];
	R[4] = A[0] * A[8] - A[2] * A[6];
	R[5] = A[2] * A[3] - A[0] * A[5];
	R[6] = A[3] * A[7] - A[4] * A[6];
	R[7] = A[1] * A[6] - A[0] * A[7];
	R[8] = A[0] * A[4] - A[1] * A[3];
	float det = A[0] * R[0] + A[3] * R[1] + A[6] * R[2];
	float inv_det = 1 / det;
	for (int i = 0; i < 9; i++)	R[i] *= inv_det;
	return det;
}
//



PD_Simulation_Cuda::PD_Simulation_Cuda(std::string model_path, float step_time)
{
	dt = step_time;
	gravity = 10;

	// 位置约束
	stiffness_position = 1000;

	stiffness_bending = 20;

	// 四面体约束
	stiffness_stretch = 2000;

	damping_coefficient = 0.99;
	// 实际测试迭代次数越多，约束会越强
	iterations_per_frame = 50;
	rho = 0.9992;

	// 加载模型
	load_model(model_path);

	// 预分配空间
	preMalloc();
}

void PD_Simulation_Cuda::update()
{
	// Jacobi迭代

	float omega = 1.0;

	// 计算Sn
	cuda_calculate_Sn(damping_coefficient, dt);

	// 迭代
	for (int iter = 0; iter < iterations_per_frame; iter++)
	{
		// 求解 w * Ac^T * (R - F)
		cuda_calculate_tet();

		// 距离约束求解
		cuda_calculate_position_constraint_force(stiffness_position);


		// 更新位置 切比雪夫加速收敛速度
		omega = 4.0 / (4.0 - rho * rho * omega);
		cuda_calculate_positions(omega, dt);
	}

	// 更新速度
	cuda_calculate_velocity(dt);

}

void PD_Simulation_Cuda::draw()
{
#ifdef DRAW_LINES
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glBindVertexArray(VAO);
	glDrawElements(GL_TRIANGLES, tet_number * 12, GL_UNSIGNED_INT, 0);
	glBindVertexArray(0);
#endif
#ifdef DRAW_PHONG
	cuda_update_normals();

	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glBindVertexArray(VAO);
	glDrawElements(GL_TRIANGLES, m_numFaces * 3, GL_UNSIGNED_INT, 0);
	glBindVertexArray(0);
#endif
#ifdef DRAW_SURFACE
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glBindVertexArray(VAO);
	glDrawElements(GL_TRIANGLES, m_numFaces * 3, GL_UNSIGNED_INT, 0);
	glBindVertexArray(0);
#endif
}

void PD_Simulation_Cuda::addFixedPoint(glm::vec3 pos, float radius)
{
	for (int i = 0; i < vertices_number; i++)
	{
		if (fixed[i] > 10) continue;
		float dis = glm::distance(glm::vec3(positions[i * 3 + 0], positions[i * 3 + 1], positions[i * 3 + 2]), pos);
		if (dis < radius)
		{
			fixed[i] = 10000000000;
		}
	}
}

void PD_Simulation_Cuda::addFixedPoint(int index)
{
	if (index >= 0 && index < vertices_number)
		fixed[index] = 10000000000;
}

void PD_Simulation_Cuda::resetPositionConstraint()
{
	position_constraint_indices.clear();
	position_constraint_targets.clear();

	cudaFree(pos_constraint_indices_cuda);
	cudaFree(pos_constraint_targets_cuda);

	memset(add_position_constraint_index_flag, 0, sizeof(bool) * vertices_number);
}

void PD_Simulation_Cuda::addPositionConstraint(d_uint index, const Vector3r& target)
{
	if (add_position_constraint_index_flag[index] == true) return;

	position_constraint_indices.push_back(index);

	position_constraint_targets.push_back(target.x());
	position_constraint_targets.push_back(target.y());
	position_constraint_targets.push_back(target.z());

	add_position_constraint_index_flag[index] = true;
}

void PD_Simulation_Cuda::addPositionConstraint(const Vector3r& target, float radius)
{
	for (int i = 0; i < vertices_number; i++)
	{
		float dis = (Vector3r(positions[i * 3 + 0], positions[i * 3 + 1], positions[i * 3 + 2]) - target).norm();
		if (dis < radius)
		{
			addPositionConstraint(i, target);
		}
	}
}

void PD_Simulation_Cuda::copyPositionConstraintToCuda()
{
	pos_constraint_num_cuda = position_constraint_indices.size();
	cudaMalloc((void**)&pos_constraint_indices_cuda, sizeof(d_uint) * pos_constraint_num_cuda);
	cudaMalloc((void**)&pos_constraint_targets_cuda, sizeof(float) * pos_constraint_num_cuda * 3);
	cudaMemcpy(pos_constraint_indices_cuda, &position_constraint_indices[0], sizeof(d_uint) * pos_constraint_num_cuda, cudaMemcpyHostToDevice);
	cudaMemcpy(pos_constraint_targets_cuda, &position_constraint_targets[0], sizeof(float) * pos_constraint_num_cuda * 3, cudaMemcpyHostToDevice);
}

void PD_Simulation_Cuda::updatePositionFromCUDA()
{
	cudaGraphicsMapResources(1, &tet_positions_gl, 0);
	size_t size = vertex_number_cuda * 3 * sizeof(float);
	cudaGraphicsResourceGetMappedPointer((void**)&positions_cuda, &size, tet_positions_gl);		// 获取内存指针

	cudaMemcpy(positions, positions_cuda, sizeof(float) * vertices_number * 3, cudaMemcpyDeviceToHost);

	cudaGraphicsUnmapResources(1, &tet_positions_gl, 0);
}

void PD_Simulation_Cuda::preMalloc()
{
	// CPU
	tetInvD3x3 = new float[tet_number * 9];
	tetInvD3x4 = new float[tet_number * 12];
	tetVolume = new float[tet_number];
	volumeDiag = new float[tet_number];
	mass = new float[vertices_number];
	fixed = new float[vertices_number];

	memset(mass, 0, sizeof(float) * vertices_number);
	memset(volumeDiag, 0, sizeof(float) * tet_number);
	memset(fixed, 0, sizeof(float) * vertices_number);


	// GPU
	cudaMalloc((void**)&vertex_number_cuda, sizeof(int));
	cudaMalloc((void**)&tet_number_cuda, sizeof(int));
	cudaMalloc((void**)&positions_cuda, sizeof(float) * vertices_number * 3);
	cudaMalloc((void**)&last_positions_cuda, sizeof(float) * vertices_number * 3);
	cudaMalloc((void**)&old_positions_cuda, sizeof(float) * vertices_number * 3);
	cudaMalloc((void**)&prev_positions_cuda, sizeof(float) * vertices_number * 3);
	cudaMalloc((void**)&next_positions_cuda, sizeof(float) * vertices_number * 3);
	cudaMalloc((void**)&velocity_cuda, sizeof(float) * vertices_number * 3);
	cudaMalloc((void**)&external_force_cuda, sizeof(float) * vertices_number * 3);
	cudaMalloc((void**)&mass_cuda, sizeof(float) * vertices_number * 3);
	cudaMalloc((void**)&tet_draw_indices_cuda, tet_number * 12 * sizeof(unsigned int));
	cudaMalloc((void**)&volumeDiag_cuda, sizeof(float) * vertices_number);
	cudaMalloc((void**)&tetInvD3x3_cuda, sizeof(float) * tet_number * 9);
	cudaMalloc((void**)&tetInvD3x4_cuda, sizeof(float) * tet_number * 12);
	cudaMalloc((void**)&tetVolume_cuda, sizeof(float) * tet_number);
	cudaMalloc((void**)&force_cuda, sizeof(float) * vertices_number * 3);
	cudaMalloc((void**)&tet_indices_cuda, sizeof(int) * tet_number * 4);
	cudaMalloc((void**)&tet_stiffness_cuda, sizeof(float) * tet_number);
	cudaMalloc((void**)&fixed_cuda, sizeof(float) * vertices_number);
	cudaMalloc((void**)&normals_cuda, sizeof(float) * vertices_number * 3);
	cudaMalloc((void**)&tet_faces_indices_cuda, sizeof(unsigned int) * m_numFaces * 3);
}

void PD_Simulation_Cuda::load_model(std::string model_path)
{
	TetMesh tetMesh(model_path);

	//
	vertices_number = tetMesh.getVertexCount();
	tet_number = tetMesh.getElementCount();
	m_numFaces = tetMesh.getFaceCount();

	// 分配空间
	positions = new float[vertices_number * 3];
	tet_indices = new int[tet_number * 4];
	tet_indices_for_draw = new d_uint[tet_number * 12];
	tet_stiffness = new float[tet_number];
	m_face_indices = new d_uint[m_numFaces * 3];
	normal = new float[vertices_number * 3];
	// 位置约束
	add_position_constraint_index_flag = new bool[vertices_number];

	// position
	for (int i = 0; i < vertices_number; i++)
	{
		positions[i * 3 + 0] = tetMesh.vertices[i].x;
		positions[i * 3 + 1] = tetMesh.vertices[i].y;
		positions[i * 3 + 2] = tetMesh.vertices[i].z;
	}

	// tet
	for (int i = 0; i < tet_number; i++)
	{
		tet_indices[i * 4 + 0] = tetMesh.elements[i].vertexIds[0];
		tet_indices[i * 4 + 1] = tetMesh.elements[i].vertexIds[1];
		tet_indices[i * 4 + 2] = tetMesh.elements[i].vertexIds[2];
		tet_indices[i * 4 + 3] = tetMesh.elements[i].vertexIds[3];

		// OpenGL EBO
		tet_indices_for_draw[i * 12 + 0] = tet_indices[i * 4 + 0];
		tet_indices_for_draw[i * 12 + 1] = tet_indices[i * 4 + 1];
		tet_indices_for_draw[i * 12 + 2] = tet_indices[i * 4 + 2];

		tet_indices_for_draw[i * 12 + 3] = tet_indices[i * 4 + 0];
		tet_indices_for_draw[i * 12 + 4] = tet_indices[i * 4 + 1];
		tet_indices_for_draw[i * 12 + 5] = tet_indices[i * 4 + 3];

		tet_indices_for_draw[i * 12 + 6] = tet_indices[i * 4 + 0];
		tet_indices_for_draw[i * 12 + 7] = tet_indices[i * 4 + 2];
		tet_indices_for_draw[i * 12 + 8] = tet_indices[i * 4 + 3];

		tet_indices_for_draw[i * 12 + 9] = tet_indices[i * 4 + 1];
		tet_indices_for_draw[i * 12 + 10] = tet_indices[i * 4 + 2];
		tet_indices_for_draw[i * 12 + 11] = tet_indices[i * 4 + 3];

		tet_stiffness[i] = stiffness_stretch;
	}

	// face
	for (int i = 0; i < m_numFaces; i++)
	{
		m_face_indices[i * 3 + 0] = tetMesh.faces[i].vertexIds[0];
		m_face_indices[i * 3 + 1] = tetMesh.faces[i].vertexIds[1];
		m_face_indices[i * 3 + 2] = tetMesh.faces[i].vertexIds[2];
	}

	// 还没写析构函数
	//delete& tetMesh;
}

void PD_Simulation_Cuda::init()
{
	float volumeSum = 0;

	for (int i = 0; i < tet_number; i++)
	{
		int id0 = tet_indices[i * 4 + 0];
		int id1 = tet_indices[i * 4 + 1];
		int id2 = tet_indices[i * 4 + 2];
		int id3 = tet_indices[i * 4 + 3];

		// 计算四面体初始shape矩阵tetInvD3x3的逆
		// 计算shape
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

		// 计算D的逆
		tetVolume[i] = fabs(Matrix_Inverse_3(D, &tetInvD3x3[i * 9]));
		volumeSum += tetVolume[i];

		// 质量
		mass[id0] += tetVolume[i] / 4;
		mass[id1] += tetVolume[i] / 4;
		mass[id2] += tetVolume[i] / 4;
		mass[id3] += tetVolume[i] / 4;

		// Ac矩阵
		float* inv_D = &tetInvD3x3[i * 9];

		tetInvD3x4[i * 12 + 0] = -inv_D[0] - inv_D[3] - inv_D[6];
		tetInvD3x4[i * 12 + 1] = inv_D[0];
		tetInvD3x4[i * 12 + 2] = inv_D[3];
		tetInvD3x4[i * 12 + 3] = inv_D[6];

		tetInvD3x4[i * 12 + 4] = -inv_D[1] - inv_D[4] - inv_D[7];
		tetInvD3x4[i * 12 + 5] = inv_D[1];
		tetInvD3x4[i * 12 + 6] = inv_D[4];
		tetInvD3x4[i * 12 + 7] = inv_D[7];

		tetInvD3x4[i * 12 + 8] = -inv_D[2] - inv_D[5] - inv_D[8];
		tetInvD3x4[i * 12 + 9] = inv_D[2];
		tetInvD3x4[i * 12 + 10] = inv_D[5];
		tetInvD3x4[i * 12 + 11] = inv_D[8];

		// 计算对角分量 w * Ac^T * Ac
		double tmp = tetVolume[i] * tet_stiffness[i];

		volumeDiag[id0] += tetInvD3x4[i * 12 + 0] * tetInvD3x4[i * 12 + 0] * tmp;
		volumeDiag[id0] += tetInvD3x4[i * 12 + 4] * tetInvD3x4[i * 12 + 4] * tmp;
		volumeDiag[id0] += tetInvD3x4[i * 12 + 8] * tetInvD3x4[i * 12 + 8] * tmp;

		volumeDiag[id1] += tetInvD3x4[i * 12 + 1] * tetInvD3x4[i * 12 + 1] * tmp;
		volumeDiag[id1] += tetInvD3x4[i * 12 + 5] * tetInvD3x4[i * 12 + 5] * tmp;
		volumeDiag[id1] += tetInvD3x4[i * 12 + 9] * tetInvD3x4[i * 12 + 9] * tmp;

		volumeDiag[id2] += tetInvD3x4[i * 12 + 2] * tetInvD3x4[i * 12 + 2] * tmp;
		volumeDiag[id2] += tetInvD3x4[i * 12 + 6] * tetInvD3x4[i * 12 + 6] * tmp;
		volumeDiag[id2] += tetInvD3x4[i * 12 + 10] * tetInvD3x4[i * 12 + 10] * tmp;

		volumeDiag[id3] += tetInvD3x4[i * 12 + 3] * tetInvD3x4[i * 12 + 3] * tmp;
		volumeDiag[id3] += tetInvD3x4[i * 12 + 7] * tetInvD3x4[i * 12 + 7] * tmp;
		volumeDiag[id3] += tetInvD3x4[i * 12 + 11] * tetInvD3x4[i * 12 + 11] * tmp;
	}

	// 传递数据到GPU
	vertex_number_cuda = vertices_number;
	tet_number_cuda = tet_number;
	face_number_cuda = m_numFaces;

	cudaMemcpy(mass_cuda, mass, sizeof(float) * vertices_number, cudaMemcpyHostToDevice);
	cudaMemcpy(tetInvD3x3_cuda, tetInvD3x3, sizeof(float) * tet_number * 9, cudaMemcpyHostToDevice);
	cudaMemcpy(tetInvD3x4_cuda, tetInvD3x4, sizeof(float) * tet_number * 12, cudaMemcpyHostToDevice);
	cudaMemcpy(tetVolume_cuda, tetVolume, sizeof(float) * tet_number, cudaMemcpyHostToDevice);
	cudaMemcpy(tet_indices_cuda, tet_indices, sizeof(int) * tet_number * 4, cudaMemcpyHostToDevice);
	cudaMemcpy(volumeDiag_cuda, volumeDiag, sizeof(float) * vertices_number, cudaMemcpyHostToDevice);
	cudaMemcpy(tet_stiffness_cuda, tet_stiffness, sizeof(float) * tet_number, cudaMemcpyHostToDevice);
	cudaMemcpy(fixed_cuda, fixed, sizeof(float) * vertices_number, cudaMemcpyHostToDevice);
	cudaMemcpy(tet_faces_indices_cuda, m_face_indices, sizeof(unsigned int) * m_numFaces * 3, cudaMemcpyHostToDevice);

	// 初始化绘制
	initDraw();
}

void PD_Simulation_Cuda::initDraw()
{
	// 设置VAO、VBO、EBO，同时绑定OpenGL和CUDA
	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);
	glGenBuffers(1, &NVBO);
	glGenBuffers(1, &EBO);

	glBindVertexArray(VAO);

	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, vertices_number * 3 * sizeof(float), positions, GL_DYNAMIC_DRAW);
	cudaGraphicsGLRegisterBuffer(&tet_positions_gl, VBO, cudaGraphicsRegisterFlagsNone);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 3, (void*)0);
	glEnableVertexAttribArray(0);

	glBindBuffer(GL_ARRAY_BUFFER, NVBO);
	glBufferData(GL_ARRAY_BUFFER, vertices_number * 3 * sizeof(float), normal, GL_DYNAMIC_DRAW);
	cudaGraphicsGLRegisterBuffer(&tet_normals_gl, NVBO, cudaGraphicsRegisterFlagsNone);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 3, (void*)0);
	glEnableVertexAttribArray(1);

#ifdef DRAW_LINES
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, tet_number * 12 * sizeof(d_uint), tet_indices_for_draw, GL_DYNAMIC_DRAW);
	cudaGraphicsGLRegisterBuffer(&tet_indices_gl, EBO, cudaGraphicsRegisterFlagsNone);
#endif
#ifdef DRAW_PHONG
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_numFaces * 3 * sizeof(d_uint), m_face_indices, GL_DYNAMIC_DRAW);
	cudaGraphicsGLRegisterBuffer(&tet_face_indices_gl, EBO, cudaGraphicsRegisterFlagsNone);
#endif
#ifdef DRAW_SURFACE
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_numFaces * 3 * sizeof(d_uint), m_face_indices, GL_DYNAMIC_DRAW);
	cudaGraphicsGLRegisterBuffer(&tet_face_indices_gl, EBO, cudaGraphicsRegisterFlagsNone);
#endif

	glBindVertexArray(0);
}
