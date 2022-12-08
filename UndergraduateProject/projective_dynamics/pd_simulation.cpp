#include "pd_simulation.h"
#include <omp.h>


PD_Simulation::PD_Simulation(std::string model_path, PD_GlobalSolverMethod global_solver_method)
{
	globalSolverMethod = global_solver_method;

	// ���ȶ�������������Щ��������ȽϺã�����Ϊ�˷�����д������Ժ��ٸģ�
	dt = 0.013333;
	gravity = 10;
	stiffness_attachment = 200;
	stiffness_bending = 20;
	stiffness_stretch = 80;
	damping_coefficient = 0.01;
	iterations_per_frame = 10;

	// Ȼ�����ģ��
	tetMesh = new TetMesh(model_path);

	// Ȼ���tetMesh�н�����ת����simulation��ģ�Ͳ�����
	vertices_number = tetMesh->getVertexCount();

	total_mass = 1.0f;		// ��㶨���
	unit_mass = total_mass / vertices_number;

	positions.resize(vertices_number * 3);
	for (d_uint i = 0; i < vertices_number; i++)
	{
		positions[i * 3 + 0] = tetMesh->vertices[i].x;
		positions[i * 3 + 1] = tetMesh->vertices[i].y;
		positions[i * 3 + 2] = tetMesh->vertices[i].z;
	}

	velocities.resize(vertices_number * 3);
	velocities.setZero();

	std::vector<EigenTriplet> mass_triplets;
	std::vector<EigenTriplet> mass_inv_triplets;
	for (int i = 0; i < vertices_number * 3; i++)
	{
		mass_triplets.push_back(EigenTriplet(i, i, unit_mass));
		mass_inv_triplets.push_back(EigenTriplet(i, i, 1.0 / unit_mass));
	}
	mass_matrix.resize(vertices_number * 3, vertices_number * 3);
	mass_matrix.setFromTriplets(mass_triplets.begin(), mass_triplets.end());
	mass_matrix_inv.resize(vertices_number * 3, vertices_number * 3);
	mass_matrix_inv.setFromTriplets(mass_inv_triplets.begin(), mass_inv_triplets.end());

	// Ȼ���һЩVector��ʼ��
	inertia.resize(vertices_number * 3);
	external_force.resize(vertices_number * 3);

	// Ȼ������ÿ��Լ����A��B����
	setupABMatrix();

	// ����Լ��
	setupConstraints();

	// Ԥ�ȼ���һЩ����
	createLHSMatrix();
	createRHSMatrix();
}

void PD_Simulation::createLHSMatrix()
{
	// Aq = b

	system_matrix_jacobi = mass_matrix / (dt * dt);

	for (Constraint* c : constraints)
	{
		EigenSparseMatrix S_i, A_i;
		d_float w_i = c->stiffness;

		if (c->constraintType == ConstraintType::ATTACHMENT)
			A_i = A_attachment;
		else if (c->constraintType == ConstraintType::SPRING)
			A_i = A_spring;
		else if (c->constraintType == ConstraintType::TETVOLUME)
			A_i = A_tetVolume;

		S_i = createSMatrix(c);

		system_matrix_jacobi += w_i * S_i.transpose() * A_i * A_i * S_i;	// A_i.transpose() == A_i
	}

	// LLT�ֽ�ϵͳ����
	if (globalSolverMethod == PD_GlobalSolverMethod::LLT)
		system_matrix_llt.compute(system_matrix_jacobi);
}

void PD_Simulation::createRHSMatrix()
{
	for (Constraint* c : constraints)
	{
		EigenSparseMatrix A_i, B_i;
		if (c->constraintType == ConstraintType::ATTACHMENT)
			A_i = A_attachment, B_i = B_attachment;
		else if (c->constraintType == ConstraintType::SPRING)
			A_i = A_spring, B_i = B_spring;
		else if (c->constraintType == ConstraintType::TETVOLUME)
			A_i = A_tetVolume, B_i = B_tetVolume;

		d_float w_i = c->stiffness;
		EigenSparseMatrix S_i = createSMatrix(c);

		c->global_RHS = w_i * S_i.transpose() * A_i * B_i;
	}
}

void PD_Simulation::update()
{
	// ���¹�����
	calculateInertia();

	// ��������
	calculateExternalForce();

	// s_n, q_n, q_(n+1), M * s_n / (dt * dt) 
	EigenVectorX s_n = inertia + dt * dt * mass_matrix_inv * external_force;
	EigenVectorX q_n = positions;
	EigenVectorX q_n1 = s_n;
	EigenVectorX Ms_ndtdt = mass_matrix * s_n / (dt * dt);

#ifdef OUTPUT_SIMULATION_TIME
	double t1;
	double t_local = 0, t_global = 0;
	std::cout << "iterations: " << iterations_per_frame;
#endif

#ifdef USE_OMP
	// ���߳�
	omp_set_num_threads(4);
#endif

	// ����
	for (d_uint iter = 0; iter < iterations_per_frame; iter++)
	{
		// ��ʼ�� Ax = b �е� b
		EigenVectorX b = Ms_ndtdt;

#ifdef OUTPUT_SIMULATION_TIME
		t1 = glfwGetTime();
#endif

		// ���ȴ�������Լ����local 
#ifdef USE_OMP
#pragma omp parallel for shared(q_n1, b)
		for (int i = 0; i < constraints.size(); i++)
		{
			Constraint* c = constraints[i];
			EigenVectorX p;

			c->solve(p, q_n1);

			EigenVectorX add_b = c->global_RHS * p;
#pragma omp critical
			{
				b += add_b;
			}
		}
#else
		for (Constraint* c : constraints)
		{
			EigenVectorX p;
			c->solve(p, q_n1);

			// �ۼ� b
			b += c->global_RHS * p;
		}
#endif

#ifdef OUTPUT_SIMULATION_TIME
		t_local += glfwGetTime() - t1;
#endif


#ifdef OUTPUT_SIMULATION_TIME
		t1 = glfwGetTime();
#endif

		// Ȼ����� Ax = b��global
		if (globalSolverMethod == PD_GlobalSolverMethod::LLT)
		{
			q_n1 = system_matrix_llt.solve(b);
		}
		else if (globalSolverMethod == PD_GlobalSolverMethod::JACOBI)
		{
			// �����и����⣬jacobiҪ�������ٴΣ���ʱ������1�� [PBUG]
			q_n1 = jacobiSolver(q_n1, b);
		}


#ifdef OUTPUT_SIMULATION_TIME
		t_global += glfwGetTime() - t1;
#endif
	}

#ifdef OUTPUT_SIMULATION_TIME
	std::cout << "  local time: " << t_local << "  global time: " << t_global << std::endl;
#endif

	// ����λ�ú��ٶ�
	velocities = (q_n1 - q_n) / dt;
	positions = q_n1;

	// �ٶ�����
	dampVelocity();

	// ģ�����
	d_float ground_y = -2;
	for (d_uint i = 0; i < vertices_number; i++)
	{
		if (positions[i * 3 + 1] < ground_y)
		{
			// ǿ�ư������ƹ�ȥ
			positions[i * 3 + 1] = ground_y;
			velocities[i * 3 + 1] = -velocities[i * 3 + 1];
			velocities[i * 3 + 0] = 0;
			velocities[i * 3 + 2] = 0;
		}
	}
}


void PD_Simulation::setupABMatrix()
{
	// B = A

	std::vector<EigenTriplet> triplets;


	// -------------------- A_attachment ----------------------
	triplets.clear();

	d_float av1 = 1;

	triplets.push_back(EigenTriplet(0, 0, av1));
	triplets.push_back(EigenTriplet(1, 1, av1));
	triplets.push_back(EigenTriplet(2, 2, av1));

	A_attachment.resize(3, 3);
	A_attachment.setFromTriplets(triplets.begin(), triplets.end());
	B_attachment.resize(3, 3);
	B_attachment = A_attachment;

	// -------------------- A_spring ----------------------
	triplets.clear();

	d_float sv1 = 0.5;
	d_float sv2 = -0.5;

	triplets.push_back(EigenTriplet(0, 0, sv1));
	triplets.push_back(EigenTriplet(1, 1, sv1));
	triplets.push_back(EigenTriplet(2, 2, sv1));
	triplets.push_back(EigenTriplet(3, 3, sv1));
	triplets.push_back(EigenTriplet(4, 4, sv1));
	triplets.push_back(EigenTriplet(5, 5, sv1));

	triplets.push_back(EigenTriplet(0, 3, sv2));
	triplets.push_back(EigenTriplet(1, 4, sv2));
	triplets.push_back(EigenTriplet(2, 5, sv2));
	triplets.push_back(EigenTriplet(3, 0, sv2));
	triplets.push_back(EigenTriplet(4, 1, sv2));
	triplets.push_back(EigenTriplet(5, 2, sv2));

	A_spring.resize(6, 6);
	A_spring.setFromTriplets(triplets.begin(), triplets.end());
	B_spring.resize(6, 6);
	B_spring = A_spring;

	// -------------------- A_tetVolume ----------------------
	triplets.clear();

	d_float tv1 = 2.0f / 3.0f;
	d_float tv2 = -1.0f / 3.0f;

	/**
	* ������Լ����A�����������£�
	*	tv1 * I4	|	tv2 * I4	|	tv2 * I4
	*	----------------------------------------
	*	tv2 * I4	|	tv1 * I4	|	tv2 * I4
	*	----------------------------------------
	*	tv2 * I4	|	tv2 * I4	|	tv1 * I4
	*/
	triplets.push_back(EigenTriplet(0, 0, tv1));	triplets.push_back(EigenTriplet(0, 4, tv2));	triplets.push_back(EigenTriplet(0, 8, tv2));
	triplets.push_back(EigenTriplet(1, 1, tv1));	triplets.push_back(EigenTriplet(1, 5, tv2));	triplets.push_back(EigenTriplet(1, 9, tv2));
	triplets.push_back(EigenTriplet(2, 2, tv1));	triplets.push_back(EigenTriplet(2, 6, tv2));	triplets.push_back(EigenTriplet(2, 10, tv2));
	triplets.push_back(EigenTriplet(3, 3, tv1));	triplets.push_back(EigenTriplet(3, 7, tv2));	triplets.push_back(EigenTriplet(3, 11, tv2));

	triplets.push_back(EigenTriplet(4, 0, tv2));	triplets.push_back(EigenTriplet(4, 4, tv1));	triplets.push_back(EigenTriplet(4, 8, tv2));
	triplets.push_back(EigenTriplet(5, 1, tv2));	triplets.push_back(EigenTriplet(5, 5, tv1));	triplets.push_back(EigenTriplet(5, 9, tv2));
	triplets.push_back(EigenTriplet(6, 2, tv2));	triplets.push_back(EigenTriplet(6, 6, tv1));	triplets.push_back(EigenTriplet(6, 10, tv2));
	triplets.push_back(EigenTriplet(7, 3, tv2));	triplets.push_back(EigenTriplet(7, 7, tv1));	triplets.push_back(EigenTriplet(7, 11, tv2));

	triplets.push_back(EigenTriplet(8, 0, tv2));	triplets.push_back(EigenTriplet(8, 4, tv2));	triplets.push_back(EigenTriplet(8, 8, tv1));
	triplets.push_back(EigenTriplet(9, 1, tv2));	triplets.push_back(EigenTriplet(9, 5, tv2));	triplets.push_back(EigenTriplet(9, 9, tv1));
	triplets.push_back(EigenTriplet(10, 2, tv2));	triplets.push_back(EigenTriplet(10, 6, tv2));	triplets.push_back(EigenTriplet(10, 10, tv1));
	triplets.push_back(EigenTriplet(11, 3, tv2));	triplets.push_back(EigenTriplet(11, 7, tv2));	triplets.push_back(EigenTriplet(11, 11, tv1));

	A_tetVolume.resize(12, 12);
	A_tetVolume.setFromTriplets(triplets.begin(), triplets.end());
	B_tetVolume.resize(12, 12);
	B_tetVolume = A_tetVolume;
}

EigenSparseMatrix PD_Simulation::createSMatrix(Constraint* c)
{
	EigenSparseMatrix S;
	std::vector<EigenTriplet> triplets;

	if (c->constraintType == ConstraintType::ATTACHMENT)
	{
		S.resize(3, vertices_number * 3);

		d_uint p0 = (dynamic_cast<AttachmentConstraint*>(c))->p0;

		triplets.push_back(EigenTriplet(0, p0 * 3 + 0, 1));
		triplets.push_back(EigenTriplet(1, p0 * 3 + 1, 1));
		triplets.push_back(EigenTriplet(2, p0 * 3 + 2, 1));
	}
	else if (c->constraintType == ConstraintType::SPRING)
	{
		S.resize(6, vertices_number * 3);

		d_uint p0 = (dynamic_cast<SpringConstraint*>(c))->p0;
		triplets.push_back(EigenTriplet(0, p0 * 3 + 0, 1));
		triplets.push_back(EigenTriplet(1, p0 * 3 + 1, 1));
		triplets.push_back(EigenTriplet(2, p0 * 3 + 2, 1));
		d_uint p1 = (dynamic_cast<SpringConstraint*>(c))->p1;
		triplets.push_back(EigenTriplet(3, p1 * 3 + 0, 1));
		triplets.push_back(EigenTriplet(4, p1 * 3 + 1, 1));
		triplets.push_back(EigenTriplet(5, p1 * 3 + 2, 1));
	}
	else if (c->constraintType == ConstraintType::TETVOLUME)
	{
		S.resize(12, vertices_number * 3);

		d_uint p0 = (dynamic_cast<TetVolumeConstraint*>(c))->p0;
		triplets.push_back(EigenTriplet(0, p0 * 3 + 0, 1));
		triplets.push_back(EigenTriplet(1, p0 * 3 + 1, 1));
		triplets.push_back(EigenTriplet(2, p0 * 3 + 2, 1));
		d_uint p1 = (dynamic_cast<TetVolumeConstraint*>(c))->p1;
		triplets.push_back(EigenTriplet(3, p1 * 3 + 0, 1));
		triplets.push_back(EigenTriplet(4, p1 * 3 + 1, 1));
		triplets.push_back(EigenTriplet(5, p1 * 3 + 2, 1));
		d_uint p2 = (dynamic_cast<TetVolumeConstraint*>(c))->p2;
		triplets.push_back(EigenTriplet(6, p2 * 3 + 0, 1));
		triplets.push_back(EigenTriplet(7, p2 * 3 + 1, 1));
		triplets.push_back(EigenTriplet(8, p2 * 3 + 2, 1));
		d_uint p3 = (dynamic_cast<TetVolumeConstraint*>(c))->p3;
		triplets.push_back(EigenTriplet(9, p3 * 3 + 0, 1));
		triplets.push_back(EigenTriplet(10, p3 * 3 + 1, 1));
		triplets.push_back(EigenTriplet(11, p3 * 3 + 2, 1));
	}

	S.setFromTriplets(triplets.begin(), triplets.end());

	return S;
}

void PD_Simulation::dampVelocity()
{
	velocities *= (1.0 - damping_coefficient);
}

void PD_Simulation::calculateInertia()
{
	inertia = positions + velocities * dt;
}

void PD_Simulation::calculateExternalForce()
{
	// ��ʱֻ������
	external_force.resize(vertices_number * 3);
	external_force.setZero();

	for (d_uint i = 0; i < vertices_number; i++)
	{
		external_force[3 * i + 1] += -gravity;
	}

	external_force = mass_matrix * external_force;
}

void PD_Simulation::clearConstraints()
{
	for (d_uint i = 0; i < constraints.size(); i++)
	{
		delete& constraints[i];
	}
	constraints.clear();
}

void PD_Simulation::setupConstraints()
{
	// ��ʱֻ����������Լ��

	for (const TetElement& e : tetMesh->elements)
	{
		constraints.push_back(
			new TetVolumeConstraint(stiffness_stretch, e.vertexIds[0], e.vertexIds[1], e.vertexIds[2], e.vertexIds[3], positions)
		);
	}

	//for (int i = 0; i < vertices_number; i++)
	//{
	//	TetVertex v = tetMesh->vertices[i];
	//	if (v.x == 1.0)
	//	{
	//		EigenVector3 x(v.x, v.y, v.z);
	//		constraints.push_back(
	//			new AttachmentConstraint(stiffness_attachment, i, x)
	//		);
	//	}
	//}
}

EigenVectorX PD_Simulation::jacobiSolver(const EigenVectorX& q_n, const EigenVectorX& b)
{
	/*
	* Jacobi ��� Ax = b �ĵ���������
	*	x_n1 = (I - D_inv * A) * x_n + D_inv * b
	* ���� D Ϊ A �ĶԽ���Ԫ����ɵĶԽǾ���
	*
	* ������ʵ������Ҫ��
	*	system_matrix_jacobi * q = b
	*/


	// return q_n1
	// q_n1 = part1 * q_n + part2

	EigenMatrixX part1(3 * vertices_number, 3 * vertices_number);
	EigenVectorX part2(3 * vertices_number);

	for (d_uint i = 0; i < 3 * vertices_number; i++)
	{
		d_float A_ii = 1.0 / system_matrix_jacobi(i, i);

		for (d_uint j = 0; j < 3 * vertices_number; j++)
		{
			// ���� part1 = I - D_inv * A
			if (i == j)
				part1(i, j) = 1.0 - A_ii * system_matrix_jacobi(i, j);
			else part1(i, j) = -A_ii * system_matrix_jacobi(i, j);
		}

		// ���� part2 = D_inv * b
		part2(i) = A_ii * b(i);
	}

	return part1 * q_n + part2;
}
