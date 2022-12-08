#pragma once

#include "../header.h"
#include "../load_model/tetMesh.h"
#include "../projective_dynamics/pd_constraint.h"
#include "../projective_dynamics/pd_cuda.h"



/*
* ������CPU+���߳���pd����Ĵ��룬����Ŀ����ʹ��
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

	// ����ϵͳ����
	void createLHSMatrix();

	// ����ÿ��Լ���� global_RHS
	void createRHSMatrix();

	// ÿһ֡����
	void update();

private:
	// Ϊÿ��Լ������A��B����
	void setupABMatrix();

	// ΪԼ������ S ����ѡ�����
	EigenSparseMatrix createSMatrix(Constraint* c);

	// �ٶ����ᣬÿһ֡���ִ��
	void dampVelocity();

	// ��������� inertia = q_n + h * v_n
	void calculateInertia();

	// ����������ÿһ֡��ʼ��ʱ��ִ��һ��
	void calculateExternalForce();

	// �������Լ��
	void clearConstraints();

	// ����Լ��
	void setupConstraints();

public:
	// jacobi��ⷽ��
	EigenVectorX jacobiSolver(const EigenVectorX& q_n, const EigenVectorX& b);



public:
	// �������
	d_float dt;							// ֡ʱ����
	d_float gravity;					// ����
	d_float stiffness_attachment;		// ����Լ��ǿ��
	d_float stiffness_stretch;
	d_float stiffness_bending;
	d_float damping_coefficient;		// �ٶ�����ϵ��
	d_uint iterations_per_frame;		// ��������

	// ģ�Ͳ���
	d_uint vertices_number;				// �ڵ���Ŀ m
	d_uint tet_number;					// ���������
	d_float total_mass;					// ������
	d_float unit_mass;					// ÿ���ڵ������
	EigenVectorX positions;				// ϵͳ״̬���ڵ�λ�ã� 3m x 1
	EigenVectorX velocities;			// �ڵ��ٶ� 3m x 1
	EigenSparseMatrix mass_matrix;		// �ڵ��������� 3m x 3m
	EigenSparseMatrix mass_matrix_inv;	// �ڵ������������ 3m x 3m


	// global��Ax = b ���㷽��
	PD_GlobalSolverMethod globalSolverMethod;
	Eigen::LLT<EigenMatrixX> system_matrix_llt;		// ϵͳ����LLT��
	EigenMatrixX system_matrix_jacobi;			// ϵͳ����Jacobi��


	// ����������ģ��
	TetMesh* tetMesh;

	// Լ������
	std::vector<Constraint*> constraints;

	// �������ʽ�е� q_n + h * v_n��
	EigenVectorX inertia;

	// ����
	EigenVectorX external_force;

	// ÿ��Լ����A��B���󣨶Գ��󣩣���Ϊ����ÿ��Լ������̫ռ�ռ䣬����ͳһ��������
	EigenSparseMatrix A_attachment;
	EigenSparseMatrix A_spring;
	EigenSparseMatrix A_tetVolume;
	EigenSparseMatrix B_attachment;
	EigenSparseMatrix B_spring;
	EigenSparseMatrix B_tetVolume;
};
