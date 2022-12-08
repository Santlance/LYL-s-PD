#ifndef __BOUNDINGSPHEREHIERARCHY_H__
#define __BOUNDINGSPHEREHIERARCHY_H__

#include "../header.h"
#include "BoundingSphere.h"
#include "kdTree.h"
#include <set>

class PointCloudBSH : public KDTree<BoundingSphere>
{

public:

	using super = KDTree<BoundingSphere>;

	PointCloudBSH();

	void init(const Vector3r* vertices, const unsigned int numVertices);
	Vector3r const& entity_position(unsigned int i) const final;
	void compute_hull(unsigned int b, unsigned int n, BoundingSphere& hull)
		const final;
	void compute_hull_approx(unsigned int b, unsigned int n, BoundingSphere& hull)
		const final;

private:
	const Vector3r* m_vertices;
	unsigned int m_numVertices;
};


class TetMeshBSH : public KDTree<BoundingSphere>
{
public:

	using super = KDTree<BoundingSphere>;

	TetMeshBSH();

	void init(const Vector3r* vertices, const unsigned int numVertices, const unsigned int* indices, const unsigned int numTets, const unsigned int* face_indices, const unsigned int numFaces, const Real tolerance);
	Vector3r const& entity_position(unsigned int i) const final;
	void compute_hull(unsigned int b, unsigned int n, BoundingSphere& hull)
		const final;
	void compute_hull_approx(unsigned int b, unsigned int n, BoundingSphere& hull)
		const final;
	void updateVertices(const Vector3r* vertices);
	void updateCom();

	bool isTriangleAFace(d_uint id1, d_uint id2, d_uint id3);

public:
	const Vector3r* m_vertices;
	unsigned int m_numVertices;
	const unsigned int* m_indices;	// indices.size = numTets * 4
	unsigned int m_numTets;
	Real m_tolerance;
	std::vector<Vector3r> m_com;	// center of mass

	// face
	unsigned int m_numFaces;
	const unsigned int* m_face_indices;
	
	// 判断三个index是否构成face的数据结构
	struct Face
	{
		d_uint x[3];
		Face(d_uint a, d_uint b, d_uint c)
		{
			x[0] = a;
			x[1] = b;
			x[2] = c;
			std::sort(x, x + 3);
		}
		bool operator == (const Face& f) const
		{
			return x[0] == f.x[0] && x[1] == f.x[1] && x[2] == f.x[2];
		}
		bool operator < (const Face& f) const
		{
			if (x[0] == f.x[0] && x[1] == f.x[1]) return x[2] < f.x[2];
			else if (x[0] == f.x[0]) return x[1] < f.x[1];
			return x[0] < f.x[0];
		}
	};
	std::set<Face> m_faceSet;

};

class BVHTest
{
public:
	using TraversalCallback = std::function <void(unsigned int node_index1, unsigned int node_index2)>;

	static void traverse(PointCloudBSH const& b1, TetMeshBSH const& b2, TraversalCallback func);
	static void traverse(PointCloudBSH const& b1, const unsigned int node_index1, TetMeshBSH const& b2, const unsigned int node_index2, TraversalCallback func);
};

#endif
