#include "CollisionDetection.h"

void CollisionDetection::initTetBSH(const Vector3r* vertices, const d_uint numVertices, const d_uint* indices, const d_uint numTets, const unsigned int* face_indices, const unsigned int numFaces, const Real tolerance)
{
	m_bshTet.init(vertices, numVertices, indices, numTets, face_indices, numFaces, tolerance);
	m_bshTet.construct();
}

void CollisionDetection::update()
{
	updateAABB();
	updateBSH();
}

void CollisionDetection::updateAABB()
{
	const Vector3r* vs = m_bshTet.m_vertices;
	const d_uint num = m_bshTet.m_numVertices;

	m_aabb.m_p[0] = vs[0];
	m_aabb.m_p[1] = vs[0];

	for (d_uint i = 1; i < num; i++)
	{
		updateAABB(vs[i], m_aabb);
	}
}

void CollisionDetection::updateBSH()
{
	m_bshTet.update();
	m_bshTet.updateCom();
}

bool CollisionDetection::collisionTest(const Vector3r& p, d_uint& tetIndex)
{
	// �����p���������aabb��Χ���ڣ��򲻿��ܷ�����ײ
	//if (!m_aabb.pointInAABB(m_aabb, p))
	//{
	//	return false;
	//}
	// �����aabb�ڲ��������BSHѰ����ײ��������


	bool isCollide = false;		// ��Ϊһ���ڵ�ֻ��һ����������ײ
	// ���ȶ�����ȱ���BSH����������
	// 
	// �ж��Ƿ����±�����ǰ�ڵ�ĺ���
	std::function<bool(d_uint, d_uint)> predicate = [&](d_uint node_index, d_uint depth)
	{
		if (isCollide) return false;

		const BoundingSphere& bs = m_bshTet.hull(node_index);

		// ���p�������ڲ��������������
		if ((p - bs.x()).norm() < bs.r())
		{
			return true;
		}
		return false;
	};
	//
	// ������ĳ��node��callback����
	std::function<void(d_uint, d_uint)> cb = [&](d_uint node_index, d_uint depth)
	{
		if (isCollide) return;

		auto const& node = m_bshTet.node(node_index);
		// �������Ҷ�ӽڵ�
		if (!node.is_leaf())
		{
			return;
		}
		// ���p�����������ڲ���������ײ
		tetIndex = m_bshTet.entity(m_bshTet.node(node_index).begin);
		if (insideTet(p, tetIndex))
		{
			isCollide = true;
		}
	};
	//
	// ����BSH
	m_bshTet.traverse_depth_first(predicate, cb);

	return isCollide;
}

bool CollisionDetection::collisionTestBS(const Vector3r& p, BoundingSphere& bs_)
{
	/*
	* ��������collisionTestһ��
	* ֻ����Ҷ�ӽڵ�ֻ�ж��Ƿ����ײ���ཻ
	*/
	bool isCollide = false;

	std::function<bool(d_uint, d_uint)> predicate = [&](d_uint node_index, d_uint depth)
	{
		if (isCollide) return false;

		const BoundingSphere& bs = m_bshTet.hull(node_index);

		// ���p�������ڲ��������������
		if ((p - bs.x()).norm() < bs.r())
		{
			return true;
		}
		return false;
	};
	//
	// ������ĳ��node��callback����
	std::function<void(d_uint, d_uint)> cb = [&](d_uint node_index, d_uint depth)
	{
		if (isCollide) return;

		auto const& node = m_bshTet.node(node_index);
		// �������Ҷ�ӽڵ�
		if (!node.is_leaf())
		{
			return;
		}
		// ���p������ײ���ڲ���������ײ
		const BoundingSphere& bs = m_bshTet.hull(node_index);
		if (bs.contains(p))
		{
			isCollide = true;
			bs_ = bs;
		}
	};
	//
	// ����BSH
	m_bshTet.traverse_depth_first(predicate, cb);

	return isCollide;
}

Real V(const Vector3r& p1, const Vector3r& p2, const Vector3r& p3, const Vector3r& p4)
{
	return fabs(1.0 / 6.0 * (p2 - p1).cross(p3 - p1).dot(p4 - p1));
}

bool CollisionDetection::insideTet(const Vector3r& p, d_uint tetIndex)
{
	const Vector3r& x1 = m_bshTet.m_vertices[m_bshTet.m_indices[tetIndex * 4 + 0]];
	const Vector3r& x2 = m_bshTet.m_vertices[m_bshTet.m_indices[tetIndex * 4 + 1]];
	const Vector3r& x3 = m_bshTet.m_vertices[m_bshTet.m_indices[tetIndex * 4 + 2]];
	const Vector3r& x4 = m_bshTet.m_vertices[m_bshTet.m_indices[tetIndex * 4 + 3]];

	return insideTet(p, x1, x2, x3, x4);
}

bool CollisionDetection::insideTet(const Vector3r& p, const Vector3r& x1, const Vector3r& x2, const Vector3r& x3, const Vector3r& x4)
{
	Real v = V(x1, x2, x3, x4);
	Real v1 = V(p, x1, x2, x3);
	Real v2 = V(p, x1, x2, x4);
	Real v3 = V(p, x1, x3, x4);
	Real v4 = V(p, x2, x3, x4);

	return fabs(v - (v1 + v2 + v3 + v4)) < 1e-6;
}

void CollisionDetection::updateAABB(const Vector3r& p, AABB& aabb)
{
	if (aabb.m_p[0][0] > p[0])
		aabb.m_p[0][0] = p[0];
	if (aabb.m_p[0][1] > p[1])
		aabb.m_p[0][1] = p[1];
	if (aabb.m_p[0][2] > p[2])
		aabb.m_p[0][2] = p[2];
	if (aabb.m_p[1][0] < p[0])
		aabb.m_p[1][0] = p[0];
	if (aabb.m_p[1][1] < p[1])
		aabb.m_p[1][1] = p[1];
	if (aabb.m_p[1][2] < p[2])
		aabb.m_p[1][2] = p[2];
}
