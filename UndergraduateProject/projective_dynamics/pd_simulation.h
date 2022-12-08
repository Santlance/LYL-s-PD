#pragma once

#include "../header.h"
#include "../load_model/tetMesh.h"
#include "../projective_dynamics/pd_constraint.h"
#include "../projective_dynamics/pd_cuda.h"



/*
* 这是用CPU+多线程做pd仿真的代码，本项目不再使用
*/

enum class PD_GlobalSolverMethod
{
	LLT,
	JACOBI,
};


class PD_Simulation
{
public:
	PD_Simulation(std::string model_path, PD_GlobalSolverMethod global_solver_method = PD_GlobalSolverMethod::LLT);

	// 计算系统矩阵
	void createLHSMatrix();

	// 计算每个约束的 global_RHS
	void createRHSMatrix();

	// 每一帧调用
	void update();

private:
	// 为每种约束设置A、B矩阵
	void setupABMatrix();

	// 为约束计算 S 矩阵（选择矩阵）
	EigenSparseMatrix createSMatrix(Constraint* c);

	// 速度阻尼，每一帧最后执行
	void dampVelocity();

	// 计算惯性项 inertia = q_n + h * v_n
	void calculateInertia();

	// 计算外力，每一帧开始的时候执行一次
	void calculateExternalForce();

	// 清除所有约束
	void clearConstraints();

	// 设置约束
	void setupConstraints();

public:
	// jacobi求解方法
	EigenVectorX jacobiSolver(const EigenVectorX& q_n, const EigenVectorX& b);



public:
	// 仿真参数
	d_float dt;							// 帧时间间隔
	d_float gravity;					// 重力
	d_float stiffness_attachment;		// 三个约束强度
	d_float stiffness_stretch;
	d_float stiffness_bending;
	d_float damping_coefficient;		// 速度阻尼系数
	d_uint iterations_per_frame;		// 迭代次数

	// 模型参数
	d_uint vertices_number;				// 节点数目 m
	d_uint tet_number;					// 四面体个数
	d_float total_mass;					// 总重量
	d_float unit_mass;					// 每个节点的重量
	EigenVectorX positions;				// 系统状态（节点位置） 3m x 1
	EigenVectorX velocities;			// 节点速度 3m x 1
	EigenSparseMatrix mass_matrix;		// 节点质量矩阵 3m x 3m
	EigenSparseMatrix mass_matrix_inv;	// 节点质量的逆矩阵 3m x 3m


	// global：Ax = b 计算方法
	PD_GlobalSolverMethod globalSolverMethod;
	Eigen::LLT<EigenMatrixX> system_matrix_llt;		// 系统矩阵（LLT）
	EigenMatrixX system_matrix_jacobi;			// 系统矩阵（Jacobi）


	// 四面体网格模型
	TetMesh* tetMesh;

	// 约束集合
	std::vector<Constraint*> constraints;

	// 惯性项（公式中的 q_n + h * v_n）
	EigenVectorX inertia;

	// 外力
	EigenVectorX external_force;

	// 每种约束的A、B矩阵（对称阵），因为存在每个约束里面太占空间，所以统一存在这里
	EigenSparseMatrix A_attachment;
	EigenSparseMatrix A_spring;
	EigenSparseMatrix A_tetVolume;
	EigenSparseMatrix B_attachment;
	EigenSparseMatrix B_spring;
	EigenSparseMatrix B_tetVolume;
};
