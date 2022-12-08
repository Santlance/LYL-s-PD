#include "pd_constraint.h"


// һЩ����
d_float clamp(d_float x, d_float lower, d_float upper)
{
	return std::max(lower, std::min(upper, x));
}


// --------------------- ATTACHMENT CONSTRAINT ---------------------------

AttachmentConstraint::AttachmentConstraint(d_float stiffness) :Constraint(stiffness)
{
	constraintType = ConstraintType::ATTACHMENT;
}

AttachmentConstraint::AttachmentConstraint(d_float stiffness, d_uint p0, EigenVector3 fixedPoint) : Constraint(stiffness)
{
	constraintType = ConstraintType::ATTACHMENT;
	this->p0 = p0;
	this->fixedPoint = fixedPoint;
}

void AttachmentConstraint::solve(EigenVectorX& p, const EigenVectorX& q)
{
	//
	p.resize(3);

	// ֱ�ӽ� p ����Ϊ�̶���
	p.blockVector3(0) = fixedPoint;
}


// --------------------- SPRING CONSTRAINT ---------------------------

SpringConstraint::SpringConstraint(d_float stiffness) : Constraint(stiffness)
{
	constraintType = ConstraintType::SPRING;
}

SpringConstraint::SpringConstraint(d_float stiffness, d_uint p0, d_uint p1, d_float restLength) : Constraint(stiffness)
{
	constraintType = ConstraintType::SPRING;
	this->p0 = p0;
	this->p1 = p1;
	this->restLength = restLength;
}

void SpringConstraint::solve(EigenVectorX& p, const EigenVectorX& q)
{
	// ���㵯���α䳤�ȣ����˽ڵ㶼���м俿��һ����α䳤��
	EigenVector3 a = q.blockVector3(p0) - q.blockVector3(p1);
	d_float deform = a.norm() - restLength;
	a = deform / 2 * a.normalized();

	p.blockVector3(0) = q.blockVector3(p0) - a;
	p.blockVector3(1) = q.blockVector3(p1) + a;
}


// --------------------- TET VOLUME CONSTRAINT ---------------------------

TetVolumeConstraint::TetVolumeConstraint(d_float stiffness) : Constraint(stiffness)
{
	constraintType = ConstraintType::TETVOLUME;
}

TetVolumeConstraint::TetVolumeConstraint(d_float stiffness, d_uint p0, d_uint p1, d_uint p2, d_uint p3, const EigenVectorX& q) : Constraint(stiffness)
{
	constraintType = ConstraintType::TETVOLUME;

	this->p0 = p0;
	this->p1 = p1;
	this->p2 = p2;
	this->p3 = p3;

	EigenVector3 v1 = q.blockVector3(p0) - q.blockVector3(p3);
	EigenVector3 v2 = q.blockVector3(p1) - q.blockVector3(p3);
	EigenVector3 v3 = q.blockVector3(p2) - q.blockVector3(p3);

	// dr = [p0-p3 | p1-p3 | p2-p3]
	dr.block<3, 1>(0, 0) = v1;
	dr.block<3, 1>(0, 1) = v2;
	dr.block<3, 1>(0, 2) = v3;

	// volume
	volume = 1.0 / 6.0 * std::fabs(dr.determinant());

	// dr_inv
	dr_inv = dr.inverse();
}

void TetVolumeConstraint::getDeformationGradient(EigenMatrix3& F, const EigenVectorX& q)
{
	// q: ��ǰϵͳ״̬����
	EigenVector3 v1 = q.blockVector3(p0) - q.blockVector3(p3);
	EigenVector3 v2 = q.blockVector3(p1) - q.blockVector3(p3);
	EigenVector3 v3 = q.blockVector3(p2) - q.blockVector3(p3);

	EigenMatrix3 ds;
	ds.block<3, 1>(0, 0) = v1;
	ds.block<3, 1>(0, 1) = v2;
	ds.block<3, 1>(0, 2) = v3;

	// ����α��ݶȣ�ds = F * dr  -->  F = ds * dr_inv
	F = ds * dr_inv;
}

void TetVolumeConstraint::singularValueDecomposition(EigenMatrix3& U, EigenVector3& SIGMA, EigenMatrix3& V, const EigenMatrix3& A)
{
	Eigen::JacobiSVD<EigenMatrix3> svd;
	svd.compute(A, Eigen::ComputeFullU | Eigen::ComputeFullV);

	U = svd.matrixU();
	V = svd.matrixV();
	SIGMA = svd.singularValues();

	// 
	if (U.determinant() < 0)
	{
		U.block<3, 1>(0, 2) *= -1;
		SIGMA[2] *= -1;
	}
	if (V.determinant() < 0)
	{
		V.block<3, 1>(0, 2) *= -1;
		SIGMA[2] *= -1;
	}
}

void TetVolumeConstraint::solve(EigenVectorX& p, const EigenVectorX& q)
{
	//
	p.resize(12);

	// �����α��ݶ�
	EigenMatrix3 F;
	getDeformationGradient(F, q);

	// �α��ݶȾ�������ֵ�ֽ⣨SVD��
	EigenMatrix3 U, V;
	EigenVector3 SIGMA;
	singularValueDecomposition(U, SIGMA, V, F);

	// �ж��������Ƿ�ת
	if (F.determinant() < 0.0)
	{
		// ԭ��SIGMA�������У�����С��ȡ��
		SIGMA[2] *= -1;
		// Ȼ���ٴν�������
		if (SIGMA[2] > SIGMA[1])
		{
			std::swap(SIGMA[2], SIGMA[1]);
			if (SIGMA[1] > SIGMA[0])
				std::swap(SIGMA[1], SIGMA[0]);
		}
	}

	// �����µ�SIGMA������Լ���Խ���Ԫ��ֵ
	EigenMatrix3 SIGMA_new;
	d_float lower = 0.95;
	d_float upper = 1.05;
	SIGMA_new << clamp(SIGMA(0, 0), lower, upper), 0, 0,
		0, clamp(SIGMA(1, 0), lower, upper), 0,
		0, 0, clamp(SIGMA(2, 0), lower, upper);

	// ����U��V��SIGMA_new�����µ�F
	EigenMatrix3 F_new = U * SIGMA_new * V.transpose();

	// ���� F_new �����µ������塰��״��
	EigenMatrix3 ds = F_new * dr;

	// ���ͶӰλ��
	EigenVector3 center = (q.blockVector3(p0) + q.blockVector3(p1) + q.blockVector3(p2) + q.blockVector3(p3)) / 4.0;
	p.blockVector3(3) = center - (ds.col(0) + ds.col(1) + ds.col(2)) / 4.0;
	p.blockVector3(0) = p.blockVector3(3) + ds.col(0);
	p.blockVector3(1) = p.blockVector3(3) + ds.col(1);
	p.blockVector3(2) = p.blockVector3(3) + ds.col(2);
}

