#pragma once

#include "../header.h"


/*
* 这是用CPU+多线程做pd仿真的代码，本项目不再使用
*/

enum ConstraintType 
{
	ATTACHMENT,		// 位置约束
	SPRING,			// 弹簧约束
	TETVOLUME,		// 四面体体积约束
};


class Constraint
{
public:
	ConstraintType constraintType;
	d_float stiffness;
	// 记录全局求解中每个约束的 (w_i * S_i^T * A_i^T * B_i) 部分 
	EigenSparseMatrix global_RHS;

	Constraint(d_float stiffness) :stiffness(stiffness) {}
	/*
	* 原本我加了这一行，导致constraintType一直无法从子类正常赋值，调试了我一晚上，因此留下这个做个纪念
	Constraint(const Constraint& other) :stiffness(other.stiffness) {}
	*/

	/**
	* 求解约束，
	*	p: 待计算的值
	*	q: 当前系统状态向量
	*/
	virtual void solve(EigenVectorX& p, const EigenVectorX& q) {}
};

class AttachmentConstraint :public Constraint
{
public:
	d_uint p0;					// 约束点的索引
	EigenVector3 fixedPoint;	// 约束位置

	AttachmentConstraint(d_float stiffness);
	AttachmentConstraint(d_float stiffness, d_uint p0, EigenVector3 fixedPoint);

	// 求解约束
	virtual void solve(EigenVectorX& p, const EigenVectorX& q);
};

class SpringConstraint : public Constraint
{
public:
	d_uint p0, p1;				// 约束点索引
	d_float restLength;			// 弹簧原长

	SpringConstraint(d_float stiffness);
	SpringConstraint(d_float stiffness, d_uint p0, d_uint p1, d_float restLength);

	// 求解约束
	virtual void solve(EigenVectorX& p, const EigenVectorX& q);
};

class TetVolumeConstraint :public Constraint
{
public:
	d_uint p0, p1, p2, p3;		// 约束点索引
	EigenMatrix3 dr;			// [p0-p3 | p1-p3 | p2-p3]，描述四面体的“形状”
	EigenMatrix3 dr_inv;
	d_float volume;				// 四面体体积

	TetVolumeConstraint(d_float stiffness);
	// 参数q：系统状态（每个节点的位置）
	TetVolumeConstraint(d_float stiffness, d_uint p0, d_uint p1, d_uint p2, d_uint p3, const EigenVectorX& q);

	// 求解约束
	/*
	* 简要说明一下求解四面体体积约束的过程。
	* 四面体形变梯度 F，包含了四面体各种形状变化，比如旋转、拉伸等，我们把 F 看成是 F = F' * R
	* 其中R是旋转变化，F'是其它变化，因为要使得四面体约束能量最小，所以我们的目的是要把 F' 这个成分给消去
	* 
	* 考虑到 R 是一个正交矩阵，所以 (F * F^T) 实际上已经不包含旋转成分了，所以我们要把 F 奇异值分解 F = U * SIGMA * V^T
	* 然后把 SIGMA 主对角线元素（其实就是 F * F^T 的特征值开根号）约束在 1 的附近，再计算一个新的 F_new，此时的 F_new 只包含旋转成分
	* 再用 F_new 去计算 p
	*/
	virtual void solve(EigenVectorX& p, const EigenVectorX& q);

private:
	// 求解形变梯度：ds = F * dr  -->  F = ds * dr_inv
	void getDeformationGradient(EigenMatrix3& F, const EigenVectorX& q);
	// 奇异值分解
	void singularValueDecomposition(EigenMatrix3& U, EigenVector3& SIGMA, EigenMatrix3& V, const EigenMatrix3& A);
};
