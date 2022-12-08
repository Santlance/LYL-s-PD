#ifndef __KDTREE_H__
#define __KDTREE_H__

#include <vector>
#include <functional>
#include <algorithm>
#include <numeric>
#include <queue>
#include <iostream>

#include "../header.h"

#include <array>
#include <list>


template <typename HullType>
class KDTree
{
public:

	// 用于遍历kdtree
	// 其中Predicate函数用于决定是否遍历当前节点的子节点
	// Callback决定如何处理遍历到的当前节点
	using TraversalPredicate = std::function<bool(unsigned int node_index, unsigned int depth)>;
	using TraversalCallback = std::function <void(unsigned int node_index, unsigned int depth)>;
	using TraversalPriorityLess = std::function<bool(std::array<int, 2> const& nodes)>;

	struct Node
	{
		Node(unsigned int b_, unsigned int n_)
			: children({ { -1, -1 } })
			, begin(b_), n(n_) {}

		Node() = default;

		bool is_leaf() const { return children[0] < 0 && children[1] < 0; }

		// Index of child nodes in nodes array.
		// -1 if child does not exist.
		std::array<int, 2> children;

		// m_lst[begin] - m_lst[begin + n - 1]的顶点都在这个node中
		// Index according entries in entity list.
		unsigned int begin;

		// Number of owned entries.
		unsigned int n;
	};

	struct QueueItem { unsigned int n, d; };
	using TraversalQueue = std::queue<QueueItem>;

	KDTree(std::size_t n, unsigned int maxPrimitivesPerLeaf = 1)
		: m_lst(n), m_maxPrimitivesPerLeaf(maxPrimitivesPerLeaf) {}

	virtual ~KDTree() {}

	Node const& node(unsigned int i) const { return m_nodes[i]; }
	HullType const& hull(unsigned int i) const { return m_hulls[i]; }
	unsigned int entity(unsigned int i) const { return m_lst[i]; }

	void construct();
	void traverse_depth_first(TraversalPredicate pred, TraversalCallback cb,
		TraversalPriorityLess const& pless = nullptr) const;
	void traverse_breadth_first(TraversalPredicate const& pred, TraversalCallback const& cb, unsigned int start_node = 0, TraversalPriorityLess const& pless = nullptr, TraversalQueue& pending = TraversalQueue()) const;
	void traverse_breadth_first_parallel(TraversalPredicate pred, TraversalCallback cb) const;

	// 顶点位置可能会变化，用于更新每个节点对应的hull
	void update();

protected:

	void construct(unsigned int node, AlignedBox3r const& box,
		unsigned int b, unsigned int n);
	void traverse_depth_first(unsigned int node, unsigned int depth,
		TraversalPredicate pred, TraversalCallback cb, TraversalPriorityLess const& pless) const;
	void traverse_breadth_first(TraversalQueue& pending,
		TraversalPredicate const& pred, TraversalCallback const& cb, TraversalPriorityLess const& pless = nullptr) const;

	unsigned int add_node(unsigned int b, unsigned int n);

	// 具体的HullType具体实现
	virtual Vector3r const& entity_position(unsigned int i) const = 0;
	virtual void compute_hull(unsigned int b, unsigned int n, HullType& hull) const = 0;
	virtual void compute_hull_approx(unsigned int b, unsigned int n, HullType& hull) const
	{
		compute_hull(b, n, hull);
	}

protected:

	// 这里的lst用来记录顶点索引
	std::vector<unsigned int> m_lst;
	// 1个node对应一个hull
	std::vector<Node> m_nodes;
	std::vector<HullType> m_hulls;
	// 叶子节点最多允许包含多少个entity
	unsigned int m_maxPrimitivesPerLeaf;
};

#include "kdTree.inl"

#endif
