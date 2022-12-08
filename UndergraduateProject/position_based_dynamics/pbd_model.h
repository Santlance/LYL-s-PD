#pragma once

#include "../header.h"
#include <vector>
#include "../simulation/CollisionDetection.h"
#include "../projective_dynamics/pd_simulation_cuda.h"

// ��������ͷ�ļ��໥��������������Ҫ�ֱ�include�Է�������Ԥ����Ҫ�õ�����
#include "pbd_constraint.h"
class PBD_Constraint;


class PBD_Model
{
public:
	PBD_Model() {}

	// ÿһ֡
	virtual void step(Real dt, d_uint iterations_number);

	// ��ӹ̶���
	virtual void addFixedPoint(d_uint p);

	// ��Ⱦ����
	virtual void draw();

protected:
	// ���¼��ٶȣ���������Ϊֻ��������
	virtual void updateAccelerations();

	// ����ʽŷ�����֣�λ��Ԥ��
	virtual void predictPositions(Real dt);

	// Լ��ͶӰ
	virtual void constraintsProjection(Real dt, d_uint iterations_number);

	// �����ٶ�
	virtual void updateVelocities(Real dt);


public:
	std::vector<EigenVector3> positions;
	std::vector<EigenVector3> positions_old;
	std::vector<EigenVector3> velocities;
	std::vector<EigenVector3> external_forces;
	std::vector<Real> masses;
	std::vector<Real> masses_inv;

	std::vector<PBD_Constraint*> constraints;
	// ��ײԼ��
	std::vector<PBD_Constraint*> collision_constraints;

public:
	// orientation
	std::vector<Real> o_masses;
	std::vector<Real> o_masses_inv;
	std::vector<Quaternionr> o_q;
	std::vector<Quaternionr> o_q0;
	std::vector<Quaternionr> o_q_old;
	std::vector<EigenVector3> o_omega;
	std::vector<EigenVector3> o_alpha;
};


class PBD_ClothModel :public PBD_Model
{
public:
	/*
	* ��ʼ��һ��cloth�����ϣ���άģ��
	*
	* ���Ͻڵ���ʾ��ͼ��
	*	O-------------------------------------> x��
	*	|	0	1	2	3	4
	*	|	5	6	7	8	9
	*	|	10	11	12	13	14
	*	|	15	16	17	18	19
	*	|
	*	|
	*	|
	*	|
	*	z��
	*/
	PBD_ClothModel(
		Real width, Real height,	// ��ȣ��߶�
		d_uint nCol, d_uint nRow,	// �ڵ�����������
		Real mass,					// ����ÿ���ڵ�������ÿ���ڵ�������ȣ�
		Real distanceStiffness, Real bendingStiffness
	);

	// ����
	virtual void draw();


private:
	// OpenGL
	d_uint VAO;
	d_uint VBO;
	d_uint EBO;
	std::vector<d_uint> indices;
};


enum class ThreadState
{
	in,
	out,
};

class PBD_ThreadModel :public PBD_Model
{
public:
	PBD_ThreadModel(
		Real length,	// ����
		d_uint size,	// �ڵ���Ŀ
		Real mass,		// ÿ���ڵ�����
		EigenVector3 position,
		int constraintType
	);

	// ����
	virtual void draw(Shader& shader);

	virtual void step(Real dt, d_uint iterations_number, CollisionDetection& cd, PD_Simulation_Cuda& pdc);

	virtual void constraintsProjection(Real dt, d_uint iterations_number, bool is_Second = false);

	void dampVelocities();

	int which_path(int id);
	bool can_in(int vid);

private:
	// OpenGL
	d_uint VAO;
	d_uint VBO;
	d_uint EBO;
	std::vector<d_uint> indices;

public:
	bool is_collision_detection;
	bool is_collision_tet;

	// ���
	Real thread_ds;								// ����߳�ʼ�ڵ���
	EigenVector3 head_position;					// ���ͷ�ڵ�λ��
	Real suture_path_min_ds;					// ���·����С�������
	std::vector<ThreadState> thread_state;		// ��¼����߽ڵ�״̬
	//
	unsigned int path_number;
	std::vector<std::vector<EigenVector3>> suture_path;		// ��¼���·��
	std::vector<std::vector<Real>> path_nodes_dis;			// ��¼���·���ڵ��ľ���
	std::vector<Real> path_length;
	bool is_end_reach;		// ��¼����ߵ����ڶ����ڵ��Ƿ��������
	//
	std::vector<bool> is_fixed;		// ��¼���·���еĽڵ��Ƿ�̶�״̬

	struct CTet
	{
		// x = a1p1 + a2p2 + a3p3 + a4p4
		// a1 + a2 + a3 + a4 = 1

		d_uint p1, p2, p3, p4;
		Real l1, l2, l3, l4;

		CTet(const Vector3r& x, d_uint tet_index, const CollisionDetection& cd);

		Vector3r update(const CollisionDetection& cd);
	};

	// ���·���ڵ���tet����ϵ
	std::vector<std::vector<CTet>> path_tet;


	// Ϊ�˱��ֵ�һ�η���߷��򲻱�(useless)
	Quaternionr head_q;


	// ����ͶӰԼ��
	std::vector<Quaternionr> o_q_save;
	std::vector<EigenVector3> positions_save;
	void save();
	void recover();
};
