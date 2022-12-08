#pragma once

#include "../header.h"
#include "AABB.h"
#include "ObjectArray.h"
#include "BoundingSphereHierarchy.h"


/*
* ֻʵ���˽ڵ����������������ײ
* �������������壩������ΰ�Χ�򣬷���߽ڵ������
*/
class CollisionDetection
{
public:
	CollisionDetection() { m_tolerance = 0; }

	void initTetBSH(const Vector3r* vertices, const d_uint numVertices, const d_uint* indices, const d_uint numTets, const unsigned int* face_indices, const unsigned int numFaces, const Real tolerance);

	Real getTolerance() const { return m_tolerance; }
	void setTolerance(Real val) { m_tolerance = val; }

	// ����
	void update();

	/*
	* ��ײ��⺯�������p���Ƿ���tetģ��/tet��ײ���ཻ
	* ����ཻ������true������tetIndex��ֵΪ�ཻ����������
	*/
	bool collisionTest(const Vector3r& p, d_uint& tetIndex);
	bool collisionTestBS(const Vector3r& p, BoundingSphere& bs);

	// ����Ƿ����������ײ�����Ƿ����������ڲ���
	bool insideTet(const Vector3r& p, d_uint tetIndex);
	bool insideTet(const Vector3r& p, const Vector3r& p1, const Vector3r& p2, const Vector3r& p3, const Vector3r& p4);

private:
	void updateAABB(const Vector3r& p, AABB& aabb);

	// ÿ���нڵ�λ�ñ仯ʱ������Ҫ����aabb
	void updateAABB();
	// ÿ���нڵ�λ�ñ仯ʱ������Ҫ����BSH
	void updateBSH();

public:
	Real m_tolerance;
	// ������ײ�壨���壩��������һ��AABB��Χ���ж��Ƿ������������ײ
	// ֻ����aabb��ײ�Ž��в�ΰ�Χ��BSH���ļ��
	AABB m_aabb;
	// ��ΰ�Χ��
	TetMeshBSH m_bshTet;
};
