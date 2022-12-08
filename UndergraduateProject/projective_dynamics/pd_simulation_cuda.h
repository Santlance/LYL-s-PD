#pragma once

#include "../header.h"
#include "../load_model/tetMesh.h"
#include "pd_cuda.h"

/*
* 根据学长代码改编整合而来
*/


class PD_Simulation_Cuda
{
public:
	PD_Simulation_Cuda(std::string model_path, float step_time);

	void update();

	// init在外部调用，方便初始化前配置一些别的东西
	void init();

	void draw();

	// 在一个球形区域内添加固定点
	void addFixedPoint(glm::vec3 pos, float radius);
	// 将某个索引的点添加为固定点
	void addFixedPoint(int index);

	/*
	* 位置约束：位置约束是每一帧动态变化的
	* 每次需要依次调用下面几个函数完成cuda端位置约束的更新
	*/
	void resetPositionConstraint();									// 重新设置位置约束
	void addPositionConstraint(d_uint index, const Vector3r& target);		// 添加位置约束
	void addPositionConstraint(const Vector3r& target, float radius);		// 目标位置球体邻域内的所有点添加位置约束
	void copyPositionConstraintToCuda();							// 更新cuda数据


	// 顶点位置需要实时更新，因为需要碰撞检测
	void updatePositionFromCUDA();


	// --------------- test ----------------------
	void addFixedPointTest()
	{
		int sum = 0;
		for (int i = 0; i < vertices_number; i++)
		{
			//if (positions[i * 3 + 0] < -0.8999 || positions[i * 3 + 0] > 0.8999)	// mesh.msh
			if (positions[i * 3 + 0] < -0.6399 || positions[i * 3 + 0] > 0.6399)	// mesh_narrow.msh
			//if (positions[i * 3 + 0] < -0.6599 || positions[i * 3 + 0] > 0.6599)	// mesh3.msh
			{
				addFixedPoint(i);
				sum++;
			}
		}
		printf("fixed points number: %d\n", sum);
	}
	// ------------------------------------------

private:
	void preMalloc();
	void load_model(std::string model_path);

	void initDraw();

public:
	// 模型参数
	d_uint vertices_number;
	d_uint tet_number;
	d_int* tet_indices;				// 四面体索引
	d_uint* tet_indices_for_draw;	// EBO数据
	float* tet_stiffness;			// 每个四面体的形变系数
	float* positions;				// 顶点位置
	float* mass;					// 顶点质量
	// face
	unsigned int m_numFaces;
	unsigned int* m_face_indices;
	float* normal;

	// 仿真参数
	d_float dt;							// 帧时间间隔
	d_float gravity;					// 重力
	// 三个约束强度
	d_float stiffness_position;			// 位置约束
	d_float stiffness_stretch;			// 四面体体积约束
	d_float stiffness_bending;
	d_float damping_coefficient;		// 速度阻尼系数
	d_uint iterations_per_frame;		// 迭代次数
	float rho;							// 雅各布迭代参数

	// 解算数据
	float* tetInvD3x3;			// 形变矩阵，9m
	float* tetInvD3x4;			// 用于计算对角阵	12m
	float* tetVolume;			// 四面体体积
	float* volumeDiag;			// w * Ac^T * Ac 的对角分量(w = k * vol)
	float* fixed;				// 是否固定（0表示不固定，大数表示固定）

	// 位置约束参数
	std::vector<d_uint> position_constraint_indices;
	std::vector<float> position_constraint_targets;
	bool* add_position_constraint_index_flag;			// 确保每个节点只添加一个距离约束（可以去掉吗？


	// OpenGL
	d_uint VAO;
	d_uint VBO;
	d_uint NVBO;
	d_uint EBO;
};

