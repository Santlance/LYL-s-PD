#pragma once

#include "../header.h"
#include "AABB.h"
#include "ObjectArray.h"
#include "BoundingSphereHierarchy.h"


/*
* 只实现了节点与四面体网格的碰撞
* 四面体网格（软体）构建层次包围球，缝合线节点遍历求交
*/
class CollisionDetection
{
public:
	CollisionDetection() { m_tolerance = 0; }

	void initTetBSH(const Vector3r* vertices, const d_uint numVertices, const d_uint* indices, const d_uint numTets, const unsigned int* face_indices, const unsigned int numFaces, const Real tolerance);

	Real getTolerance() const { return m_tolerance; }
	void setTolerance(Real val) { m_tolerance = val; }

	// 更新
	void update();

	/*
	* 碰撞检测函数，检测p点是否与tet模型/tet碰撞球相交
	* 如果相交，返回true，并且tetIndex赋值为相交的四面体编号
	*/
	bool collisionTest(const Vector3r& p, d_uint& tetIndex);
	bool collisionTestBS(const Vector3r& p, BoundingSphere& bs);

	// 检测是否和四面体碰撞（点是否在四面体内部）
	bool insideTet(const Vector3r& p, d_uint tetIndex);
	bool insideTet(const Vector3r& p, const Vector3r& p1, const Vector3r& p2, const Vector3r& p3, const Vector3r& p4);

private:
	void updateAABB(const Vector3r& p, AABB& aabb);

	// 每当有节点位置变化时，都需要更新aabb
	void updateAABB();
	// 每当有节点位置变化时，都需要更新BSH
	void updateBSH();

public:
	Real m_tolerance;
	// 整个碰撞体（软体）单独设置一个AABB包围盒判断是否可能与软体碰撞
	// 只有与aabb碰撞才进行层次包围球（BSH）的检测
	AABB m_aabb;
	// 层次包围球
	TetMeshBSH m_bshTet;
};
