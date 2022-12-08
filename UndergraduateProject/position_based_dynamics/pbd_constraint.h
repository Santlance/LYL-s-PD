#pragma once

#include "../header.h"
#include <vector>
#include "../simulation/BoundingSphere.h"

// 对于两个头文件相互引用类的情况，需要分别include对方，并且预声明要用到的类
#include "pbd_model.h"
class PBD_Model;
enum class ThreadState;


class PBD_Constraint
{
public:
	PBD_Constraint(const d_uint number_of_indices, Real stiffness);

	virtual bool solve(PBD_Model& model, const d_uint iter, bool is_Second = false);

public:
	// 约束顶点索引
	std::vector<d_uint> indices;
	// 约束强度
	Real stiffness;
};

class PBD_DistanceConstraint :public PBD_Constraint
{
	/*
	* 距离约束：
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
	* 二面角约束：p3-p4为公共边
	*	n1 = ((p3 - p1) x (p4 - p1)).normalize()
	*	n2 = ((p3 - p2) x (p4 - p2)).normalize()
	*	C(p1, p2, p3, p4) = arccos(n1 · n2) - rest_angle
	*
	* GitHub找到的代码是用的Simulation of Clothing with Folds and Wrinkles这篇论文的方法，
	* 我这里实现的是PBD论文原本的方法
	*
	* 实验测试论文原本的方法有问题（也有可能是我写错了），所以改成了GitHub代码的方法
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
	// 用于二次约束投影
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
	// 用于二次约束投影
	std::vector<ThreadState>& thread_state;
};


class PBD_CollisionSphereConstraint :public PBD_Constraint
{
	/*
	* 简单处理点和球碰撞的约束
	*/
public:
	PBD_CollisionSphereConstraint(
		const PBD_Model& model,
		const d_uint p1,						// 点在缝合线上的索引
		const BoundingSphere& bs,
		const Real stiffness
	);

	virtual bool solve(PBD_Model& model, const d_uint iter, bool is_Second = false);

public:
	BoundingSphere bs;				// 被碰撞的球体
};


class PBD_CollisionPointTriangleConstraint :public PBD_Constraint
{
	/*
	* 点和三角形碰撞约束
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
