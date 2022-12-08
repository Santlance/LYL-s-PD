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
	// 如果点p不在最外层aabb包围盒内，则不可能发生碰撞
	//if (!m_aabb.pointInAABB(m_aabb, p))
	//{
	//	return false;
	//}
	// 如果在aabb内部，则遍历BSH寻找碰撞的四面体


	bool isCollide = false;		// 认为一个节点只和一个四面体碰撞
	// 首先定义深度遍历BSH的两个函数
	// 
	// 判断是否往下遍历当前节点的函数
	std::function<bool(d_uint, d_uint)> predicate = [&](d_uint node_index, d_uint depth)
	{
		if (isCollide) return false;

		const BoundingSphere& bs = m_bshTet.hull(node_index);

		// 如果p点在球内部则继续往下搜索
		if ((p - bs.x()).norm() < bs.r())
		{
			return true;
		}
		return false;
	};
	//
	// 遍历到某个node的callback函数
	std::function<void(d_uint, d_uint)> cb = [&](d_uint node_index, d_uint depth)
	{
		if (isCollide) return;

		auto const& node = m_bshTet.node(node_index);
		// 不处理非叶子节点
		if (!node.is_leaf())
		{
			return;
		}
		// 如果p点在四面体内部，发生碰撞
		tetIndex = m_bshTet.entity(m_bshTet.node(node_index).begin);
		if (insideTet(p, tetIndex))
		{
			isCollide = true;
		}
	};
	//
	// 遍历BSH
	m_bshTet.traverse_depth_first(predicate, cb);

	return isCollide;
}

bool CollisionDetection::collisionTestBS(const Vector3r& p, BoundingSphere& bs_)
{
	/*
	* 大体代码和collisionTest一样
	* 只是在叶子节点只判断是否和碰撞球相交
	*/
	bool isCollide = false;

	std::function<bool(d_uint, d_uint)> predicate = [&](d_uint node_index, d_uint depth)
	{
		if (isCollide) return false;

		const BoundingSphere& bs = m_bshTet.hull(node_index);

		// 如果p点在球内部则继续往下搜索
		if ((p - bs.x()).norm() < bs.r())
		{
			return true;
		}
		return false;
	};
	//
	// 遍历到某个node的callback函数
	std::function<void(d_uint, d_uint)> cb = [&](d_uint node_index, d_uint depth)
	{
		if (isCollide) return;

		auto const& node = m_bshTet.node(node_index);
		// 不处理非叶子节点
		if (!node.is_leaf())
		{
			return;
		}
		// 如果p点在碰撞球内部，发生碰撞
		const BoundingSphere& bs = m_bshTet.hull(node_index);
		if (bs.contains(p))
		{
			isCollide = true;
			bs_ = bs;
		}
	};
	//
	// 遍历BSH
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
