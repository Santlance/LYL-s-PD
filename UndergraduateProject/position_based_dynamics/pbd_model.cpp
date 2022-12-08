#include "pbd_model.h"

Real pbd_gravity = -5.0;


void PBD_Model::updateAccelerations()
{
	for (int i = 0; i < external_forces.size(); i++)
	{
		external_forces[i] = EigenVector3(0, pbd_gravity, 0);
	}
}

void PBD_Model::predictPositions(Real dt)
{
	for (int i = 0; i < positions.size(); i++)
	{
		// 首先保存当前位置
		positions_old[i] = positions[i];

		// 然后预测下一个位置
		velocities[i] += external_forces[i] * masses_inv[i] * dt;
		positions[i] += velocities[i] * dt;
	}

	for (int i = 0; i < o_q.size(); i++)
	{
		o_q_old[i] = o_q[i];

		// 预测
		if (o_masses[i] != 0)
		{
			EigenMatrix3 invInertialW = o_masses_inv[i] * EigenMatrix3::Identity();
			EigenVector3 torque(0, 0, 0);

			o_omega[i] += dt * invInertialW * torque;
			Quaternionr angVelQ(0.0, o_omega[i][0], o_omega[i][1], o_omega[i][2]);
			o_q[i].coeffs() += dt * 0.5 * (angVelQ * o_q[i]).coeffs();
			o_q[i].normalize();
		}
	}
}

void PBD_Model::constraintsProjection(Real dt, d_uint iterations_number)
{
	for (d_uint iter = 1; iter <= iterations_number; iter++)
	{
		/*
		* 这里先进行碰撞约束投影相对来说更稳定一些
		*/
		// 碰撞约束投影
		for (PBD_Constraint* constraint : collision_constraints)
		{
			constraint->solve(*this, iter);
		}
		// 约束投影
		for (PBD_Constraint* constraint : constraints)
		{
			constraint->solve(*this, iter);
		}
	}
}

void PBD_Model::updateVelocities(Real dt)
{
	for (d_uint i = 0; i < velocities.size(); i++)
	{
		velocities[i] = (positions[i] - positions_old[i]) / dt;
	}

	for (d_uint i = 0; i < o_omega.size(); i++)
	{
		Quaternionr relRot = (o_q[i] * o_q_old[i].conjugate());
		o_omega[i] = relRot.vec() * (2.0 / dt);
	}
}

void PBD_Model::step(Real dt, d_uint iterations_number)
{
	// 1. 更新加速度（设置外力）
	updateAccelerations();
	// 2. 位置预测
	predictPositions(dt);
	// 3. 约束投影
	constraintsProjection(dt, iterations_number);
	// 4. 更新速度
	updateVelocities(dt);
}

void PBD_Model::addFixedPoint(d_uint p)
{
	masses[p] = 100000000;
	masses_inv[p] = 0;
}

void PBD_Model::draw()
{
	return;
}


PBD_ClothModel::PBD_ClothModel(Real width, Real height, d_uint nCol, d_uint nRow, Real mass, Real distanceStiffness, Real bendingStiffness)
{
	if (width <= 0 || height <= 0 || nCol < 2 || nRow < 2 || mass <= 0)
		return;

	Real dx = width / (nCol - 1);
	Real dz = height / (nRow - 1);

	for (d_uint i = 0; i < nRow; i++)
	{
		for (d_uint j = 0; j < nCol; j++)
		{
			Real x = 0.0 + j * dx;
			Real z = 0.0 + i * dz;

			positions.push_back(EigenVector3(x, 0, z));
			positions_old.push_back(EigenVector3(x, 0, z));
			velocities.push_back(EigenVector3(0, 0, 0));
			masses.push_back(mass);
			masses_inv.push_back(1 / mass);
			external_forces.push_back(EigenVector3(0, 0, 0));
		}
	}


	// ***构造约束***
	// 
	// 1. 每条边添加距离约束
	for (d_uint i = 0; i < nRow; i++)
		for (d_uint j = 0; j < nCol - 1; j++)
		{
			// 横向
			d_uint id1 = i * nCol + j;
			d_uint id2 = i * nCol + j + 1;
			constraints.push_back(new PBD_DistanceConstraint(*this, id1, id2, distanceStiffness));
		}
	for (d_uint j = 0; j < nCol; j++)
		for (d_uint i = 0; i < nRow - 1; i++)
		{
			// 纵向
			d_uint id1 = i * nCol + j;
			d_uint id2 = (i + 1) * nCol + j;
			constraints.push_back(new PBD_DistanceConstraint(*this, id1, id2, distanceStiffness));
		}
	for (d_uint i = 0; i < nRow - 1; i++)
		for (d_uint j = 0; j < nCol - 1; j++)
		{
			// 斜向
			d_uint id1 = i * nCol + j;
			d_uint id2 = (i + 1) * nCol + j + 1;
			constraints.push_back(new PBD_DistanceConstraint(*this, id1, id2, distanceStiffness));

			id1 = i * nCol + j + 1;
			id2 = (i + 1) * nCol + j;
			constraints.push_back(new PBD_DistanceConstraint(*this, id1, id2, distanceStiffness));
		}

	// 2. 添加二面角约束
	for (d_uint i = 1; i < nRow - 1; i++)
		for (d_uint j = 0; j < nCol - 1; j++)
		{
			// 横边为公共边
			d_uint id1 = i * nCol + j;
			d_uint id2 = i * nCol + j + 1;
			d_uint id3 = (i - 1) * nCol + j;
			d_uint id4 = (i + 1) * nCol + j;
			constraints.push_back(new PBD_DihedralConstraint(*this, id3, id4, id1, id2, bendingStiffness));
		}
	for (d_uint j = 1; j < nCol - 1; j++)
		for (d_uint i = 0; i < nRow - 1; i++)
		{
			// 纵边为公共边
			d_uint id1 = i * nCol + j;
			d_uint id2 = (i + 1) * nCol + j;
			d_uint id3 = i * nCol + j - 1;
			d_uint id4 = i * nCol + j + 1;
			constraints.push_back(new PBD_DihedralConstraint(*this, id3, id4, id1, id2, bendingStiffness));
		}
	for (d_uint i = 0; i < nRow - 1; i++)
		for (d_uint j = 0; j < nCol - 1; j++)
		{
			// 斜边为公共边
			d_uint id1 = i * nCol + j;
			d_uint id2 = (i + 1) * nCol + j + 1;
			d_uint id3 = i * nCol + j + 1;
			d_uint id4 = (i + 1) * nCol + j;
			constraints.push_back(new PBD_DihedralConstraint(*this, id3, id4, id1, id2, bendingStiffness));
		}


	// ***初始化渲染用到的数据***

	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);
	glGenBuffers(1, &EBO);

	// 创建EBO需要的索引数据
	indices.clear();
	for (d_uint i = 0; i < nRow - 1; i++)
		for (d_uint j = 0; j < nCol - 1; j++)
		{
			indices.push_back(i * nCol + j);
			indices.push_back(i * nCol + j + 1);
			indices.push_back((i + 1) * nCol + j + 1);

			indices.push_back(i * nCol + j);
			indices.push_back((i + 1) * nCol + j);
			indices.push_back((i + 1) * nCol + j + 1);
		}
}

void PBD_ClothModel::draw()
{
	glBindVertexArray(VAO);

	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, positions.size() * sizeof(EigenVector3), &positions[0], GL_STATIC_DRAW);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(d_uint), &indices[0], GL_STATIC_DRAW);

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(EigenVector3), (void*)0);
	glEnableVertexAttribArray(0);

	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);

	glBindVertexArray(0);
}


PBD_ThreadModel::PBD_ThreadModel(Real length, d_uint size, Real mass, EigenVector3 position, int constraintType)
{
	if (size < 2 || length <= 0) return;

	is_collision_detection = true;
	is_collision_tet = false;

	const EigenMatrix3 sutureOrientation = EigenMatrix3::Identity();
	const Real helixRadius = 0.5;
	const Real helixHeight = -5;
	const Real helixTotalAngle = 10 * M_PI;
	const EigenMatrix3 helixOrientation = EigenMatrix3::Identity();

	// 节点间距
	Real dx = length / (size - 1);
	thread_ds = dx;

	EigenVector3 initP = position;
	for (d_uint i = 0; i < size; i++)
	{
		EigenVector3 p(dx * i, 0, 0);
		if (i > 0) p.x() = p.x() - dx + dx / 10;
		p = sutureOrientation * p + initP;

		// 原本的model
		positions.push_back(p);
		positions_old.push_back(p);
		velocities.push_back(EigenVector3(0, 0, 0));
		masses.push_back(mass);
		masses_inv.push_back(1 / mass);
		external_forces.push_back(EigenVector3(0, 0, 0));

		// 
		thread_state.push_back(ThreadState::out);
		is_fixed.push_back(false);
	}

	// 初始化quaternions
	EigenVector3 from(0, 0, 1);
	for (int i = 0; i < size - 1; i++)
	{
		EigenVector3 to = (positions[i + 1] - positions[i]).normalized();
		Quaternionr dq = Quaternionr::FromTwoVectors(from, to);
		if (i > 0)
		{
			dq *= o_q[i - 1];
		}

		o_q.push_back(dq);
		o_q0.push_back(dq);
		o_q_old.push_back(dq);
		o_masses.push_back(mass);
		o_masses_inv.push_back(1.0 / mass);
		o_omega.push_back(EigenVector3(0, 0, 0));
		o_alpha.push_back(EigenVector3(0, 0, 0));

		from = to;
	}

	positions_save.resize(positions.size());
	o_q_save.resize(o_q.size());

	// 头节点质量无穷大，static
	masses[0] = mass * 10000;
	masses_inv[0] = 0;
	o_masses[0] = masses[0];
	o_masses_inv[0] = 0;

	//
	head_position = initP;
	suture_path_min_ds = 0.005;
	path_number = 0;
	suture_path.resize(512);
	path_nodes_dis.resize(512);
	is_end_reach = false;
	path_length.resize(512);
	for (int i = 0; i < path_length.size(); i++)
		path_length[i] = 0;
	path_tet.resize(512);


	// ***构造约束***
	//
	if (constraintType == 0)
	{
		for (int i = 0; i < size - 1; i++)
		{
			// i, i + 1
			constraints.push_back(new PBD_StretchShearConstraint(
				*this, i, i + 1, i,
				EigenVector3(0.8, 0.8, 0.8),
				//EigenVector3(0.1, 0.1, 0.1),
				this->thread_state
			));
		}
		for (int i = 0; i < size - 2; i++)
		{
			constraints.push_back(new PBD_BendTwistConstraint(
				*this, i, i + 1,
				EigenVector3(0.1, 0.1, 0.1),
				//EigenVector3(0.3, 0.3, 0.3),
				this->thread_state
			));
		}
	}
	else if (constraintType == 1)
	{
		for (int i = 0; i < size - 1; i++)
		{
			constraints.push_back(new PBD_DistanceConstraint(
				*this, i, i + 1, 0.3
			));
			if (i < size - 2)
			{
				constraints.push_back(new PBD_DistanceConstraint(
					*this, i, i + 2, 0.3
				));
			}
		}
	}



	// ***初始化渲染用到的数据***

	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);
	glGenBuffers(1, &EBO);

	for (int i = 0; i < size - 1; i++)
	{
		indices.push_back(i);
		indices.push_back(i + 1);
	}
}

void PBD_ThreadModel::draw(Shader& shader)
{
	glBindVertexArray(VAO);

	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, positions.size() * sizeof(EigenVector3), &positions[0], GL_STATIC_DRAW);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(d_uint), &indices[0], GL_STATIC_DRAW);

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(EigenVector3), (void*)0);
	glEnableVertexAttribArray(0);

	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	shader.use();
	shader.setInt("color", 0);

#if defined(DRAW_LINES) || defined(DRAW_SURFACE)
	glLineWidth(2);
	glPointSize(10.0);
	glDrawElements(GL_LINES, indices.size(), GL_UNSIGNED_INT, 0);

	for (int i = 0; i < positions.size(); i++)
	{
		if (is_fixed[i])
			shader.setInt("color", 2);
		else if (thread_state[i] == ThreadState::out)
			shader.setInt("color", 0);
		else shader.setInt("color", 1);
		glDrawArrays(GL_POINTS, i, 1);
	}
#endif
#ifdef DRAW_PHONG
	glLineWidth(10);
	glDrawElements(GL_LINES, indices.size(), GL_UNSIGNED_INT, 0);
#endif

	glBindVertexArray(0);
}

void PBD_ThreadModel::dampVelocities()
{
	float coeff = 0.998;
	for (d_uint i = 0; i < velocities.size(); i++)
	{
		velocities[i] *= coeff;
	}

	for (d_uint i = 0; i < o_omega.size(); i++)
	{
		o_omega[i] *= coeff;
	}
}

int PBD_ThreadModel::which_path(int id)
{
	if (id < 0 || id >= positions.size() || path_number == 0 || thread_state[id] == ThreadState::out) return -1;

	int cur = 0;
	int path = path_number;
	while (path > 0)
	{
		while (cur < positions.size() && thread_state[cur] == ThreadState::out) cur++;
		path--;
		while (cur != id && cur < positions.size() && thread_state[cur] == ThreadState::in) cur++;
		if (cur == id) break;
	}
	return path;
}

bool PBD_ThreadModel::can_in(int vid)
{
	int path = which_path(vid - 1);
	if (path == -1) return false;

	// path里面的平均节点间距过小，则当前节点不可进入
	int right = vid, left = vid - 1;
	while (left > 0 && thread_state[left] == ThreadState::in) left--;
	if (left == 0) return true;		// 穿刺过程中可以进入
	if (path_length[path] / (right - left) < thread_ds)
		return false;

	return true;
}

void PBD_ThreadModel::save()
{
	for (int i = 0; i < positions.size(); i++)
	{
		positions_save[i] = positions[i];
	}
	for (int i = 0; i < o_q.size(); i++)
	{
		o_q_save[i] = o_q[i];
	}
}

void PBD_ThreadModel::recover()
{
	for (int i = 0; i < positions.size(); i++)
	{
		positions[i] = positions_save[i];
	}
	for (int i = 0; i < o_q.size(); i++)
	{
		o_q[i] = o_q_save[i];
	}
}

void PBD_ThreadModel::step(Real dt, d_uint iterations_number, CollisionDetection& cd, PD_Simulation_Cuda& pdc)
{
	// 1. 更新加速度（设置外力）
	updateAccelerations();

	// 1.5 damp try
	dampVelocities();

	// 2. 位置预测
	predictPositions(dt);
	positions[0] = head_position;
	//o_q[0] = head_q;


	// 2.5 碰撞检测
	if (is_collision_detection)
	{
		collision_constraints.clear();
		for (int i = 1; i < positions.size(); i++)
		{
			// 在软体外部边缘的节点不进行碰撞检测，因为要不影响它们进入软体
			int ignore_cnt = 2;
			if (thread_state[i] == ThreadState::out)
			{
				bool flag = false;
				for (int j = -ignore_cnt; j <= ignore_cnt; j++)
					if ((i + j >= 0 && thread_state[i + j] == ThreadState::in) || (i + j < positions.size() && thread_state[i + j] == ThreadState::in))
						flag = true;
				if (flag) continue;
			}

			// 只对软体外部的节点碰撞检测
			if (thread_state[i] == ThreadState::out)
			{
				// 碰撞检测为检测是否和四面体相交
				if (is_collision_tet)
				{
					d_uint tetIndex;
					if (cd.collisionTest(positions[i], tetIndex))
					{
						//printf("Collision!   TetIndex: %d\n", tetIndex);
						const unsigned int id1 = cd.m_bshTet.m_indices[tetIndex * 4 + 0];
						const unsigned int id2 = cd.m_bshTet.m_indices[tetIndex * 4 + 1];
						const unsigned int id3 = cd.m_bshTet.m_indices[tetIndex * 4 + 2];
						const unsigned int id4 = cd.m_bshTet.m_indices[tetIndex * 4 + 3];
						const Vector3r& p1 = cd.m_bshTet.m_vertices[id1];
						const Vector3r& p2 = cd.m_bshTet.m_vertices[id2];
						const Vector3r& p3 = cd.m_bshTet.m_vertices[id3];
						const Vector3r& p4 = cd.m_bshTet.m_vertices[id4];
						const Real stiffness = 1.0;
						/*
						* PBD_CollisionPointTriangleConstraint处理约束时只会约束*点的运动轨迹穿过三角形平面*的情况，
						* 因此如果position[i]在四面体内部而position_old[i]在四面体外部，也就是发生了碰撞，那么这个点
						* 的运动轨迹必然只会穿过四面体的某一个三角形面。
						*
						* 虽然上面的分析是对的，但是存在一些问题，比如处理不了点直接穿过整个四面体的情况
						*/

						if (cd.m_bshTet.isTriangleAFace(id1, id2, id3))
						{
							collision_constraints.push_back(new PBD_CollisionPointTriangleConstraint(*this, i, p1, p2, p3, stiffness));
						}
						if (cd.m_bshTet.isTriangleAFace(id1, id2, id4))
						{
							collision_constraints.push_back(new PBD_CollisionPointTriangleConstraint(*this, i, p1, p2, p4, stiffness));
						}
						if (cd.m_bshTet.isTriangleAFace(id1, id3, id4))
						{
							collision_constraints.push_back(new PBD_CollisionPointTriangleConstraint(*this, i, p1, p3, p4, stiffness));
						}
						if (cd.m_bshTet.isTriangleAFace(id2, id3, id4))
						{
							collision_constraints.push_back(new PBD_CollisionPointTriangleConstraint(*this, i, p2, p3, p4, stiffness));
						}
					}
				}
				// 碰撞检测为检测是否和球碰撞体相交
				// 实际测试这种碰撞检测方式能够更好地处理缝合线与软体相交的边界处，效果更加稳定
				else
				{
					BoundingSphere bs;
					const Real stiffness = 0.3;
					if (cd.collisionTestBS(positions[i], bs))
					{
						collision_constraints.push_back(new PBD_CollisionSphereConstraint(*this, i, bs, stiffness));
					}
				}
			}
		}
	}



	// 2.9 二次约束投影——保存
	save();

	// 3. 约束投影
	constraintsProjection(dt, iterations_number);
	positions[0] = head_position;
	//o_q[0] = head_q;




	//  ++++++++++++++ 根据 positions 和 positions_old 给四面体网格模型添加力 +++++++++++++++++++
	Real stiffness_tmp = 0.25;
	//if (is_end_reach) stiffness_tmp = 0.3;
	pdc.resetPositionConstraint();

	std::vector<Vector3r> position_force_vectors;	// 记录缝合线牵引软体时，给每个节点带来的位置约束力向量
	position_force_vectors.resize(pdc.vertices_number);
	for (Vector3r& v : position_force_vectors)
		v = Vector3r(0, 0, 0);

	d_uint path_iter = 0;	// 记录下面的遍历过程中当前节点在第几条缝合路径
	bool flag = false;
	for (int i = positions.size() - 1; i > 0; i--)	// 缝合线末尾节点对应缝合路径编号最小
	{
		if (thread_state[i] == ThreadState::in)
		{
			flag = true;
			d_uint tetIndex;
			if (cd.collisionTest(positions[i], tetIndex))
			{
				const unsigned int id1 = cd.m_bshTet.m_indices[tetIndex * 4 + 0];
				const unsigned int id2 = cd.m_bshTet.m_indices[tetIndex * 4 + 1];
				const unsigned int id3 = cd.m_bshTet.m_indices[tetIndex * 4 + 2];
				const unsigned int id4 = cd.m_bshTet.m_indices[tetIndex * 4 + 3];
				Vector3r d = positions[i] - positions_old[i];

				/*
				* 实验结果表明，越早建立的缝合路径对软体的牵扯越弱
				* （也有可能是，除了最新建立的，其它的都很弱）。
				* 因此这里根据缝合路径编号再添加一个系数
				*/
				Real coeff = 1.0;
				//if (is_end_reach && path_iter < path_number - 1) coeff = 1.5;

				coeff = exp((float)i / positions.size() - 1);

				position_force_vectors[id1] += coeff * stiffness_tmp * d;
				position_force_vectors[id2] += coeff * stiffness_tmp * d;
				position_force_vectors[id3] += coeff * stiffness_tmp * d;
				position_force_vectors[id4] += coeff * stiffness_tmp * d;
			}
		}
		else
		{
			if (flag)
			{
				flag = false;
				path_iter++;
			}
		}
	}

	for (int i = 0; i < pdc.vertices_number; i++)
	{
		if ((position_force_vectors[i]).norm() > 1e-6)
		{
			pdc.addPositionConstraint(i, cd.m_bshTet.m_vertices[i] + position_force_vectors[i]);
		}
	}

	pdc.copyPositionConstraintToCuda();
	// --------------------------------------------------------------------------------------



	// 二次约束投影
	//recover();
	//constraintsProjection(dt, iterations_number, true);
	//positions[0] = head_position;




	// ++++++++++++++++++ 0 处理缝合路径 ++++++++++++++++++

	// 0.1 首先处理头节点，通过碰撞检测来判断头节点是否在软体内部
	d_uint tetIndex;
	if (cd.collisionTest(positions[0], tetIndex))
	{
		if (thread_state[0] == ThreadState::out)
		{
			// 头节点从out变成in，新建一条缝合路径
			path_number++;
		}
		thread_state[0] = ThreadState::in;
	}
	else
	{
		thread_state[0] = ThreadState::out;
	}

	// 0.2 然后根据头节点更新碰撞路径
	if (thread_state[0] == ThreadState::in)
	{
		if (suture_path[path_number - 1].empty())
		{
			suture_path[path_number - 1].push_back(head_position);

			cd.collisionTest(head_position, tetIndex);
			path_tet[path_number - 1].push_back(CTet(head_position, tetIndex, cd));
		}
		else
		{
			Real dis = (head_position - suture_path[path_number - 1][suture_path[path_number - 1].size() - 1]).norm();
			if (dis > suture_path_min_ds)
			{
				suture_path[path_number - 1].push_back(head_position);
				path_nodes_dis[path_number - 1].push_back(dis);
				path_length[path_number - 1] += dis;

				cd.collisionTest(head_position, tetIndex);
				path_tet[path_number - 1].push_back(CTet(head_position, tetIndex, cd));
			}
		}
	}

	// 0.3 然后更新每个节点的 in/out 状态
	for (int i = 1; i < positions.size(); i++)
	{
		// 固定的节点不更新节点状态
		if (is_fixed[i]) continue;

		// 跟前又跟后
		//if (((i > 0 && thread_state[i - 1] == ThreadState::in) || (i < positions.size() - 1 && thread_state[i + 1] == ThreadState::in)) && cd.collisionTest(positions[i], tetIndex))
		//{
		//	// 前一个或后一个节点在软体里面时可以变成in
		//	thread_state[i] = ThreadState::in;
		//}
		//else if (((i > 0 && thread_state[i - 1] == ThreadState::out) || (i < positions.size() - 1 && thread_state[i + 1] == ThreadState::out)) && !cd.collisionTest(positions[i], tetIndex))
		//{
		//	// 前一个或后一个节点在软体外面时可以变成out
		//	thread_state[i] = ThreadState::out;
		//}

		// 跟前不跟后
		if ((i > 0 && thread_state[i - 1] == ThreadState::in) && cd.collisionTest(positions[i], tetIndex))
		{
			if (!can_in(i)) continue;

			// positions[i]在缝合路径进入点附近才可以进入
			int path = which_path(i - 1);
			//printf("%d %d\n", path, path_number);
			Vector3r insert_point = suture_path[path][0];
			if ((insert_point - positions[i]).norm() > thread_ds * 5)
			{
				continue;
			}

			// 保证两个缝合路径之间存在out的节点
			if (i < positions.size() - 1 && thread_state[i + 1] != ThreadState::out) continue;
			thread_state[i] = ThreadState::in;
		}
		else if ((i > 0 && thread_state[i - 1] == ThreadState::out) && !cd.collisionTest(positions[i], tetIndex))
		{
			thread_state[i] = ThreadState::out;
		}
	}

	// 0.4 然后对于 in 的节点吸附到缝合路径上
	/*
	* 这里的吸附直接把position移到了缝合路径上，(同时也要改变quaternion的值???)
	*/
	unsigned int left = 0, right = 0;
	for (int k = path_number - 1; k >= 0; k--)
	{
		const std::vector<EigenVector3>& path = suture_path[k];
		const std::vector<Real>& path_dis = path_nodes_dis[k];
		/*
		* 从最后一条缝合路径开始（对应缝合线最前面的部分）往前遍历
		* 对于每一段缝合路径，找到缝合线中处于缝合路径上的区间 [left, right)，
		* 然后根据距离来依附在缝合路径上
		*/
		left = right;
		while (left < positions.size() && thread_state[left] == ThreadState::out) left++;
		right = left;
		while (right < positions.size() && thread_state[right] == ThreadState::in) right++;

		//printf("%d :    %d %d\n", k, left, right);

		// positions[left, right) 依附到 *整条* path 上
		unsigned int cur = path.size() - 1;
		Real d = 0;
		Real ds = path_length[k] / (right - left - 1);	// 平均ds距离依附一个节点
		positions[left] = path[cur];
		for (int i = left + 1; i < right; i++)
		{
			d = 0;
			while (d < ds && cur > 0)
			{
				cur--;
				d += path_dis[cur];
			}
			positions[i] = path[cur];
		}
		// 对应更新quaternion（凡是两端节点有动过的，对应的quaternion都要更新，[left-1, right-1]）
		//EigenVector3 from = left > 0 ? (positions[left] - positions[left - 1]).normalized() : (positions[1] - positions[0]).normalized();
		//for (int i = std::max((int)left, 1); i <= std::min((int)right - 1, (int)positions.size() - 2); i++)
		//{
		//	EigenVector3 to = (positions[i + 1] - positions[i]).normalized();
		//	o_q[i] = Quaternionr::FromTwoVectors(from, to);
		//	o_q[i] *= o_q[i - 1];
		//	o_q_old[i] = o_q[i];
		//	from = to;
		//}

		// 当ds大于某个值时，固定缝合路径上的节点状态
		//float ratio = 3.0;	// 实验测出的数据
		//if (k == 0) ratio = 2.2;
		float ratio = 3.0;	// 实验测出的数据
		if (k == 0) ratio = 2.5;

		if (ds > thread_ds * ratio && left > 2)
		{
			// 如果上一段缝合路径上的节点还没固定，这一段就跳过
			bool flag = true;
			if (k > 0)
			{
				int q = right;
				while (q < positions.size() - 1 && thread_state[q] == ThreadState::out) q++;
				if (!is_fixed[q]) flag = false;
			}

			if (flag)
			{
				// 固定
				for (int i = left; i < right; i++)
				{
					is_fixed[i] = true;
				}
				// 也要把缝合路径后面out的节点固定
				if (k > 0)
				{
					int q = right;
					while (q < positions.size() && thread_state[q] == ThreadState::out)
					{
						is_fixed[q] = true;
						q++;
					}
				}

				// test
				if (k == 1) is_fixed[left - 1] = true;
			}
		}
	}

	// 0.5 如果缝合线最后一个节点进入了软体，那么它固定在软体表面处
	if (is_end_reach == false && thread_state[thread_state.size() - 1] == ThreadState::in)
	{
		is_end_reach = true;
	}
	if (is_end_reach)
	{
		is_fixed[thread_state.size() - 1] = true;
		thread_state[thread_state.size() - 1] = ThreadState::in;
		Vector3r x = suture_path[0][0];
		x.y() += thread_ds / 2;
		positions[positions.size() - 1] = x;
	}

	// 0.6 根据四面体模型更新缝合路径位置
	for (int i = 0; i < path_number; i++)
	{
		std::vector<EigenVector3>& path = suture_path[i];
		for (int j = 0; j < path.size(); j++)
		{
			path[j] = path_tet[i][j].update(cd);
		}
	}



	// （PBD）4. 最后更新速度
	updateVelocities(dt);
}

void PBD_ThreadModel::constraintsProjection(Real dt, d_uint iterations_number, bool is_Second)
{
	for (d_uint iter = 1; iter <= iterations_number; iter++)
	{
		/*
		* 这里先进行碰撞约束投影相对来说更稳定一些
		*/
		// 碰撞约束投影
		for (PBD_Constraint* constraint : collision_constraints)
		{
			constraint->solve(*this, iter);
		}
		// 约束投影
		for (PBD_Constraint* constraint : constraints)
		{
			if (is_Second) constraint->solve(*this, iter, true);
			else constraint->solve(*this, iter);
		}
		positions[0] = head_position;
	}
}

PBD_ThreadModel::CTet::CTet(const Vector3r& x, d_uint tet_index, const CollisionDetection& cd)
{
	p1 = cd.m_bshTet.m_indices[tet_index * 4 + 0];
	p2 = cd.m_bshTet.m_indices[tet_index * 4 + 1];
	p3 = cd.m_bshTet.m_indices[tet_index * 4 + 2];
	p4 = cd.m_bshTet.m_indices[tet_index * 4 + 3];
	const Vector3r& x1 = cd.m_bshTet.m_vertices[p1];
	const Vector3r& x2 = cd.m_bshTet.m_vertices[p2];
	const Vector3r& x3 = cd.m_bshTet.m_vertices[p3];
	const Vector3r& x4 = cd.m_bshTet.m_vertices[p4];

	Real a1 = x1.x(), b1 = x1.y(), c1 = x1.z();
	Real a2 = x2.x(), b2 = x2.y(), c2 = x2.z();
	Real a3 = x3.x(), b3 = x3.y(), c3 = x3.z();
	Real a4 = x4.x(), b4 = x4.y(), c4 = x4.z();

	EigenMatrix3 A;
	A << a1 - a4, a2 - a4, a3 - a4,
		b1 - b4, b2 - b4, b3 - b4,
		c1 - c4, c2 - c4, c3 - c4;
	EigenVector3 b(x.x() - a4, x.y() - b4, x.z() - c4);
	EigenVector3 res = A.inverse() * b;

	this->l1 = res.x();
	this->l2 = res.y();
	this->l3 = res.z();
	this->l4 = 1 - this->l1 - this->l2 - this->l3;

	//printf("+++:  %f %f %f    %d\n", x.x(), x.y(), x.z(),tet_index);
	//Vector3r t = l1 * x1 + l2 * x2 + l3 * x3 + l4 * x4;
	//printf("---:  %f %f %f\n", t.x(), t.y(), t.z());
}

Vector3r PBD_ThreadModel::CTet::update(const CollisionDetection& cd)
{
	const Vector3r& x1 = cd.m_bshTet.m_vertices[p1];
	const Vector3r& x2 = cd.m_bshTet.m_vertices[p2];
	const Vector3r& x3 = cd.m_bshTet.m_vertices[p3];
	const Vector3r& x4 = cd.m_bshTet.m_vertices[p4];

	return l1 * x1 + l2 * x2 + l3 * x3 + l4 * x4;
}
