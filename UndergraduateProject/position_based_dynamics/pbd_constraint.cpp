#include "pbd_constraint.h"


PBD_Constraint::PBD_Constraint(const d_uint number_of_indices, Real stiffness)
{
	indices.resize(number_of_indices);
	this->stiffness = stiffness;
}

bool PBD_Constraint::solve(PBD_Model& model, const d_uint iter, bool is_Second)
{
	return false;
}

PBD_DistanceConstraint::PBD_DistanceConstraint(
	const PBD_Model& model,
	const d_uint p1, const d_uint p2, const Real stiffness) :PBD_Constraint(2, stiffness)
{
	indices[0] = p1;
	indices[1] = p2;

	rest_length = (model.positions[p1] - model.positions[p2]).norm();
}

bool PBD_DistanceConstraint::solve(PBD_Model& model, const d_uint iter, bool is_Second)
{
	EigenVector3& x1 = model.positions[indices[0]];
	EigenVector3& x2 = model.positions[indices[1]];
	const Real invMass1 = model.masses_inv[indices[0]];
	const Real invMass2 = model.masses_inv[indices[1]];

	// solve
	Real wSum = invMass1 + invMass2;
	if (wSum == 0) return false;

	EigenVector3 n = x2 - x1;
	Real d = n.norm();
	n.normalize();

	EigenVector3 corr, corr1, corr2;
	corr = stiffness * n * (d - rest_length) / wSum;
	corr1 = invMass1 * corr;
	corr2 = -invMass2 * corr;

	// update
	if (invMass1 != 0) x1 += corr1;
	if (invMass2 != 0) x2 += corr2;

	return true;
}

PBD_DihedralConstraint::PBD_DihedralConstraint(
	const PBD_Model& model,
	const d_uint p1, const d_uint p2, const d_uint p3, const d_uint p4,
	const Real stiffness) :PBD_Constraint(4, stiffness)
{
	indices[0] = p1;
	indices[1] = p2;
	indices[2] = p3;
	indices[3] = p4;

	const EigenVector3& x1 = model.positions[p1];
	const EigenVector3& x2 = model.positions[p2];
	const EigenVector3& x3 = model.positions[p3];
	const EigenVector3& x4 = model.positions[p4];


	EigenVector3 n1 = (x3 - x1).cross(x4 - x1);
	EigenVector3 n2 = (x4 - x2).cross(x3 - x2);
	n1 /= n1.squaredNorm();
	n2 /= n2.squaredNorm();

	n1.normalize();
	n2.normalize();

	Real dot = n1.dot(n2);
	if (dot < -1.0) dot = -1.0;
	if (dot > 1.0) dot = 1.0;

	rest_angle = acos(dot);
}

bool PBD_DihedralConstraint::solve(PBD_Model& model, const d_uint iter, bool is_Second)
{
	EigenVector3& x1 = model.positions[indices[0]];
	EigenVector3& x2 = model.positions[indices[1]];
	EigenVector3& x3 = model.positions[indices[2]];
	EigenVector3& x4 = model.positions[indices[3]];

	const Real invMass1 = model.masses_inv[indices[0]];
	const Real invMass2 = model.masses_inv[indices[1]];
	const Real invMass3 = model.masses_inv[indices[2]];
	const Real invMass4 = model.masses_inv[indices[3]];


	if ((invMass1 == 0 && invMass2 == 0) || (x4 - x3).norm() < 1e-6)
		return false;

	EigenVector3 e = x4 - x3;
	Real elen = e.norm();
	Real invElen = static_cast<Real>(1.0) / elen;

	EigenVector3 n1 = (x3 - x1).cross(x4 - x1);
	EigenVector3 n2 = (x4 - x2).cross(x3 - x2);
	n1 /= n1.squaredNorm();
	n2 /= n2.squaredNorm();

	EigenVector3 d1 = elen * n1;
	EigenVector3 d2 = elen * n2;
	EigenVector3 d3 = (x1 - x4).dot(e) * invElen * n1 + (x2 - x4).dot(e) * invElen * n2;
	EigenVector3 d4 = (x3 - x1).dot(e) * invElen * n1 + (x3 - x2).dot(e) * invElen * n2;

	n1.normalize();
	n2.normalize();

	Real dot = n1.dot(n2);
	if (dot < -1.0) dot = -1.0;
	if (dot > 1.0) dot = 1.0;

	Real phi = acos(dot);
	//Real phi = (-0.6981317 * dot * dot - 0.8726646) * dot + 1.570796;	// fast approximation
	Real lambda = invMass1 * d1.squaredNorm() + invMass2 * d2.squaredNorm() + invMass3 * d3.squaredNorm() + invMass4 * d4.squaredNorm();

	if (lambda == 0.0) return false;

	// 这里似乎有个稳定性的问题，GitHub代码有提到，还需要看相应论文才能理解为什么

	lambda = (phi - rest_angle) / lambda * stiffness;

	if (n1.cross(n2).dot(e) > 0.0) lambda = -lambda;

	EigenVector3 corr1 = -invMass1 * lambda * d1;
	EigenVector3 corr2 = -invMass2 * lambda * d2;
	EigenVector3 corr3 = -invMass3 * lambda * d3;
	EigenVector3 corr4 = -invMass4 * lambda * d4;

	// 更新
	x1 += corr1;
	x2 += corr2;
	x3 += corr3;
	x4 += corr4;

	return true;
}

PBD_StretchShearConstraint::PBD_StretchShearConstraint(
	const PBD_Model& model,
	const d_uint p1, const d_uint p2,
	const d_uint quaternion,
	const EigenVector3& stiffness,
	std::vector<ThreadState>& ts) :PBD_Constraint(3, 0), thread_state(ts)
{
	indices[0] = p1;
	indices[1] = p2;
	indices[2] = quaternion;
	this->stiffness = stiffness;
	this->rest_length = (model.positions[p1] - model.positions[p2]).norm();
}

bool PBD_StretchShearConstraint::solve(PBD_Model& model, const d_uint iter, bool is_Second)
{
	EigenVector3& x0 = model.positions[indices[0]];
	EigenVector3& x1 = model.positions[indices[1]];
	Quaternionr& q0 = model.o_q[indices[2]];

	Real invMass0 = model.masses_inv[indices[0]];
	Real invMass1 = model.masses_inv[indices[1]];
	Real invMassq0 = model.o_masses_inv[indices[2]];


	// 二次约束投影
	if (is_Second)
	{
		//if (thread_state[indices[0]] == ThreadState::in && thread_state[indices[1]] == ThreadState::in)
		//	return false;

		int q = 2;
		bool flag0 = true, flag1 = true;
		for (int i = 0; i < q; i++)
		{
			if (indices[0] > i && thread_state[indices[0] - (i + 1)] == ThreadState::out)
				flag0 = false;
			if (indices[0] < thread_state.size() - (i + 1) && thread_state[indices[0] + (i + 1)] == ThreadState::out)
				flag0 = false;
			if (indices[1] > i && thread_state[indices[1] - (i + 1)] == ThreadState::out)
				flag1 = false;
			if (indices[1] < thread_state.size() - (i + 1) && thread_state[indices[1] + (i + 1)] == ThreadState::out)
				flag1 = false;
		}

		if (thread_state[indices[0]] == ThreadState::in && flag0)	// 内部边缘节点仍然要投影
			invMass0 = 0;
		if (thread_state[indices[1]] == ThreadState::in && flag1)
			invMass1 = 0;
		if (invMass0 == 0 && invMass1 == 0) invMassq0 = 0;
	}


	// solve
	EigenVector3 d3(
		2.0 * (q0.x() * q0.z() + q0.w() * q0.y()),
		2.0 * (q0.y() * q0.z() - q0.w() * q0.x()),
		q0.w() * q0.w() - q0.x() * q0.x() - q0.y() * q0.y() + q0.z() * q0.z()
	);
	EigenVector3 gamma = (x1 - x0) / rest_length - d3;
	gamma /= (invMass1 + invMass0) / rest_length + invMassq0 * 4.0 * rest_length + 1.0e-6;
	for (int i = 0; i < 3; i++) gamma[i] *= stiffness[i];

	EigenVector3 corr0 = invMass0 * gamma;
	EigenVector3 corr1 = -invMass1 * gamma;

	Quaternionr q_e_3_bar(q0.z(), -q0.y(), q0.x(), -q0.w());
	Quaternionr corrq0 = Quaternionr(0.0, gamma.x(), gamma.y(), gamma.z()) * q_e_3_bar;
	corrq0.coeffs() *= 2.0 * invMassq0 * rest_length;


	//if (is_Second && indices[0] == 0 && indices[1] == 1)
	//{
	//	std::cout << x0 << std::endl << x1 << std::endl;
	//	if (invMass1 > 0)
	//		printf("XXX:   %f, %f, %f\n", corr1.x(), corr1.y(), corr1.z());
	//	else puts("");
	//}


	// update
	if (invMass0 != 0) x0 += corr0;
	if (invMass1 != 0) x1 += corr1;
	if (invMassq0 != 0)
	{
		q0.coeffs() += corrq0.coeffs();
		q0.normalize();
	}

	return true;
}

PBD_BendTwistConstraint::PBD_BendTwistConstraint(
	const PBD_Model& model,
	const d_uint quaternion1, const d_uint quaternion2,
	const EigenVector3& stiffness,
	std::vector<ThreadState>& ts) :PBD_Constraint(2, 0), thread_state(ts)
{
	indices[0] = quaternion1;
	indices[1] = quaternion2;
	this->stiffness = stiffness;

	const Quaternionr& q1_0 = model.o_q[quaternion1];
	const Quaternionr& q2_0 = model.o_q[quaternion2];

	rest_DarbouxVector = q1_0.conjugate() * q2_0;

	Quaternionr omega_plus, omega_minus;
	omega_plus.coeffs() = rest_DarbouxVector.coeffs() + Quaternionr(1, 0, 0, 0).coeffs();
	omega_minus.coeffs() = rest_DarbouxVector.coeffs() - Quaternionr(1, 0, 0, 0).coeffs();
	if (omega_minus.squaredNorm() > omega_plus.squaredNorm())
		rest_DarbouxVector.coeffs() *= -1.0;
}

bool PBD_BendTwistConstraint::solve(PBD_Model& model, const d_uint iter, bool is_Second)
{
	Quaternionr& q0 = model.o_q[indices[0]];
	Quaternionr& q1 = model.o_q[indices[1]];
	Real invMassq0 = model.o_masses_inv[indices[0]];
	Real invMassq1 = model.o_masses_inv[indices[1]];


	// 二次约束投影
	if (is_Second)
	{
		if (thread_state[indices[0]] == ThreadState::in && thread_state[indices[0] + 1] == ThreadState::in)
			invMassq0 = 0;
		if (thread_state[indices[1]] == ThreadState::in && thread_state[indices[1] + 1] == ThreadState::in)
			invMassq1 = 0;
	}


	// solve
	Quaternionr omega = q0.conjugate() * q1;   //darboux vector

	Quaternionr omega_plus;
	omega_plus.coeffs() = omega.coeffs() + rest_DarbouxVector.coeffs();     //delta Omega with -Omega_0
	omega.coeffs() = omega.coeffs() - rest_DarbouxVector.coeffs();                 //delta Omega with + omega_0
	if (omega.squaredNorm() > omega_plus.squaredNorm()) omega = omega_plus;

	for (int i = 0; i < 3; i++) omega.coeffs()[i] *= stiffness[i] / (invMassq0 + invMassq1 + 1.0e-6);
	omega.w() = 0.0;    //discrete Darboux vector does not have vanishing scalar part

	Quaternionr corrq0 = q1 * omega;
	Quaternionr corrq1 = q0 * omega;
	corrq0.coeffs() *= invMassq0;
	corrq1.coeffs() *= -invMassq1;

	// update
	if (invMassq0 != 0)
	{
		q0.coeffs() += corrq0.coeffs();
		q0.normalize();
	}
	if (invMassq1 != 0)
	{
		q1.coeffs() += corrq1.coeffs();
		q1.normalize();
	}

	return true;
}

PBD_CollisionSphereConstraint::PBD_CollisionSphereConstraint(
	const PBD_Model& model, const d_uint p1,
	const BoundingSphere& bs, const Real stiffness) :PBD_Constraint(1, stiffness)
{
	indices[0] = p1;
	this->bs.x() = bs.x();
	this->bs.r() = bs.r();
}

bool PBD_CollisionSphereConstraint::solve(PBD_Model& model, const d_uint iter, bool is_Second)
{
	/*
	* 这里的处理为直接把点投影到球面上，处理方式比较粗暴
	* 最好优化一下
	*/

	EigenVector3& x1 = model.positions[indices[0]];
	const Real invMass1 = model.masses_inv[indices[0]];

	// solve
	// 投影到球面上
	Real r = bs.r();
	Vector3r dir = (x1 - bs.x()).normalized();
	Vector3r dest = bs.x() + r * dir;
	Vector3r corr1 = stiffness * (dest - x1);

	// update
	if (invMass1 != 0) x1 += corr1;

	return true;
}

PBD_CollisionPointTriangleConstraint::PBD_CollisionPointTriangleConstraint(
	const PBD_Model& model,
	const d_uint p,
	const Vector3r& p1, const Vector3r& p2, const Vector3r& p3,
	const Real stiffness) :PBD_Constraint(1, stiffness)
{
	indices[0] = p;
	this->p1 = p1;
	this->p2 = p2;
	this->p3 = p3;
}

bool insideTriangle(const Vector3r& p, const Vector3r& x1, const Vector3r& x2, const Vector3r& x3)
{
	Vector3r a = (x2 - x1).cross(p - x1);
	Vector3r b = (x3 - x2).cross(p - x2);
	Vector3r c = (x1 - x3).cross(p - x3);
	return a.dot(b) >= 0 && b.dot(c) >= 0;
}

bool PBD_CollisionPointTriangleConstraint::solve(PBD_Model& model, const d_uint iter, bool is_Second)
{
	Vector3r& x = model.positions[indices[0]];
	const Vector3r& x_old = model.positions_old[indices[0]];
	const Real invMass = model.masses_inv[indices[0]];

	// solve

	/*
	// 将 x 约束到 x_old --> x 穿过三角形的位置
	Vector3r d = x - x_old;
	if (d.norm() < 1e-6) return false;
	// 假设交点为 x_old + td , 令交点在三角形平面上（注意这个交点必须在三角形内部）
	Vector3r n = (p2 - p1).cross(p3 - p1);
	Real t = (p1.dot(n) - x_old.dot(n)) / d.dot(n);
	if (t < 0 || t > 1) return false;

	Vector3r pi = x_old + t * d;
	if (!insideTriangle(pi, p1, p2, p3)) return false;

	Vector3r corr = stiffness * (pi - x);
	*/

	// 将 x 约束到三角形上最近的点
	Vector3r n = (p2 - p1).cross(p3 - p1);
	Real t = (p1.dot(n) - x.dot(n)) / n.dot(n);
	Vector3r pi = x + t * n;
	Vector3r corr = stiffness * (pi - x);

	// update
	if (invMass != 0)
	{
		x += corr;
	}
	return true;
}
