#pragma once

#include "../header.h"
#include <vector>
#include "../simulation/BoundingSphere.h"

// ��������ͷ�ļ��໥��������������Ҫ�ֱ�include�Է�������Ԥ����Ҫ�õ�����
#include "pbd_model.h"
class PBD_Model;
enum class ThreadState;


class PBD_Constraint
{
public:
	PBD_Constraint(const d_uint number_of_indices, Real stiffness);

	virtual bool solve(PBD_Model& model, const d_uint iter, bool is_Second = false);

public:
	// Լ����������
	std::vector<d_uint> indices;
	// Լ��ǿ��
	Real stiffness;
};

class PBD_DistanceConstraint :public PBD_Constraint
{
	/*
	* ����Լ����
	*	C(p1, p2) = |p1 - p2| - rest_length
	*/
public:
	PBD_DistanceConstraint(
		const PBD_Model& model,
		const d_uint p1, const d_uint p2,
		const Real stiffness
	);

	virtual bool solve(PBD_Model& model, const d_uint iter, bool is_Second = false);

public:
	Real rest_length;
};

class PBD_DihedralConstraint :public PBD_Constraint
{
	/*
	* �����Լ����p3-p4Ϊ������
	*	n1 = ((p3 - p1) x (p4 - p1)).normalize()
	*	n2 = ((p3 - p2) x (p4 - p2)).normalize()
	*	C(p1, p2, p3, p4) = arccos(n1 �� n2) - rest_angle
	*
	* GitHub�ҵ��Ĵ������õ�Simulation of Clothing with Folds and Wrinkles��ƪ���ĵķ�����
	* ������ʵ�ֵ���PBD����ԭ���ķ���
	*
	* ʵ���������ԭ���ķ��������⣨Ҳ�п�������д���ˣ������Ըĳ���GitHub����ķ���
	*/
public:
	PBD_DihedralConstraint(
		const PBD_Model& model,
		const d_uint p1, const d_uint p2, const d_uint p3, const d_uint p4,
		const Real stiffness
	);

	virtual bool solve(PBD_Model& model, const d_uint iter, bool is_Second = false);

public:
	Real rest_angle;
};

class PBD_StretchShearConstraint :public PBD_Constraint
{
public:
	PBD_StretchShearConstraint(
		const PBD_Model& model,
		const d_uint p1, const d_uint p2,
		const d_uint quaternion,
		const EigenVector3& stiffness,
		std::vector<ThreadState>& ts
	);

	virtual bool solve(PBD_Model& model, const d_uint iter, bool is_Second = false);

public:
	Real rest_length;
	EigenVector3 stiffness;
	// ���ڶ���Լ��ͶӰ
	std::vector<ThreadState>& thread_state;
};

class PBD_BendTwistConstraint :public PBD_Constraint
{
public:
	PBD_BendTwistConstraint(
		const PBD_Model& model,
		const d_uint quaternion1, const d_uint quaternion2,
		const EigenVector3& stiffness,
		std::vector<ThreadState>& ts
	);

	virtual bool solve(PBD_Model& model, const d_uint iter, bool is_Second = false);

public:
	Quaternionr rest_DarbouxVector;
	EigenVector3 stiffness;
	// ���ڶ���Լ��ͶӰ
	std::vector<ThreadState>& thread_state;
};


class PBD_CollisionSphereConstraint :public PBD_Constraint
{
	/*
	* �򵥴���������ײ��Լ��
	*/
public:
	PBD_CollisionSphereConstraint(
		const PBD_Model& model,
		const d_uint p1,						// ���ڷ�����ϵ�����
		const BoundingSphere& bs,
		const Real stiffness
	);

	virtual bool solve(PBD_Model& model, const d_uint iter, bool is_Second = false);

public:
	BoundingSphere bs;				// ����ײ������
};


class PBD_CollisionPointTriangleConstraint :public PBD_Constraint
{
	/*
	* �����������ײԼ��
	*/
public:
	PBD_CollisionPointTriangleConstraint(
		const PBD_Model& model,
		const d_uint p,
		const Vector3r& p1, const Vector3r& p2, const Vector3r& p3,
		const Real stiffness
	);

	virtual bool solve(PBD_Model& model, const d_uint iter, bool is_Second = false);

public:
	Vector3r p1, p2, p3;
};
