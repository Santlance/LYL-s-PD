#pragma once

#include "../header.h"


/*
* ������CPU+���߳���pd����Ĵ��룬����Ŀ����ʹ��
*/

enum ConstraintType 
{
	ATTACHMENT,		// λ��Լ��
	SPRING,			// ����Լ��
	TETVOLUME,		// ���������Լ��
};


class Constraint
{
public:
	ConstraintType constraintType;
	d_float stiffness;
	// ��¼ȫ�������ÿ��Լ���� (w_i * S_i^T * A_i^T * B_i) ���� 
	EigenSparseMatrix global_RHS;

	Constraint(d_float stiffness) :stiffness(stiffness) {}
	/*
	* ԭ���Ҽ�����һ�У�����constraintTypeһֱ�޷�������������ֵ����������һ���ϣ�������������������
	Constraint(const Constraint& other) :stiffness(other.stiffness) {}
	*/

	/**
	* ���Լ����
	*	p: �������ֵ
	*	q: ��ǰϵͳ״̬����
	*/
	virtual void solve(EigenVectorX& p, const EigenVectorX& q) {}
};

class AttachmentConstraint :public Constraint
{
public:
	d_uint p0;					// Լ���������
	EigenVector3 fixedPoint;	// Լ��λ��

	AttachmentConstraint(d_float stiffness);
	AttachmentConstraint(d_float stiffness, d_uint p0, EigenVector3 fixedPoint);

	// ���Լ��
	virtual void solve(EigenVectorX& p, const EigenVectorX& q);
};

class SpringConstraint : public Constraint
{
public:
	d_uint p0, p1;				// Լ��������
	d_float restLength;			// ����ԭ��

	SpringConstraint(d_float stiffness);
	SpringConstraint(d_float stiffness, d_uint p0, d_uint p1, d_float restLength);

	// ���Լ��
	virtual void solve(EigenVectorX& p, const EigenVectorX& q);
};

class TetVolumeConstraint :public Constraint
{
public:
	d_uint p0, p1, p2, p3;		// Լ��������
	EigenMatrix3 dr;			// [p0-p3 | p1-p3 | p2-p3]������������ġ���״��
	EigenMatrix3 dr_inv;
	d_float volume;				// ���������

	TetVolumeConstraint(d_float stiffness);
	// ����q��ϵͳ״̬��ÿ���ڵ��λ�ã�
	TetVolumeConstraint(d_float stiffness, d_uint p0, d_uint p1, d_uint p2, d_uint p3, const EigenVectorX& q);

	// ���Լ��
	/*
	* ��Ҫ˵��һ��������������Լ���Ĺ��̡�
	* �������α��ݶ� F�������������������״�仯��������ת������ȣ����ǰ� F ������ F = F' * R
	* ����R����ת�仯��F'�������仯����ΪҪʹ��������Լ��������С���������ǵ�Ŀ����Ҫ�� F' ����ɷָ���ȥ
	* 
	* ���ǵ� R ��һ�������������� (F * F^T) ʵ�����Ѿ���������ת�ɷ��ˣ���������Ҫ�� F ����ֵ�ֽ� F = U * SIGMA * V^T
	* Ȼ��� SIGMA ���Խ���Ԫ�أ���ʵ���� F * F^T ������ֵ�����ţ�Լ���� 1 �ĸ������ټ���һ���µ� F_new����ʱ�� F_new ֻ������ת�ɷ�
	* ���� F_new ȥ���� p
	*/
	virtual void solve(EigenVectorX& p, const EigenVectorX& q);

private:
	// ����α��ݶȣ�ds = F * dr  -->  F = ds * dr_inv
	void getDeformationGradient(EigenMatrix3& F, const EigenVectorX& q);
	// ����ֵ�ֽ�
	void singularValueDecomposition(EigenMatrix3& U, EigenVector3& SIGMA, EigenMatrix3& V, const EigenMatrix3& A);
};
