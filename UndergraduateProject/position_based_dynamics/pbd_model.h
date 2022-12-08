#pragma once

#include "../header.h"
#include <vector>
#include "../simulation/CollisionDetection.h"
#include "../projective_dynamics/pd_simulation_cuda.h"

// 对于两个头文件相互引用类的情况，需要分别include对方，并且预声明要用到的类
#include "pbd_constraint.h"
class PBD_Constraint;


class PBD_Model
{
public:
	PBD_Model() {}

	// 每一帧
	virtual void step(Real dt, d_uint iterations_number);

	// 添加固定点
	virtual void addFixedPoint(d_uint p);

	// 渲染绘制
	virtual void draw();

protected:
	// 更新加速度（这里设置为只有重力）
	virtual void updateAccelerations();

	// 半隐式欧拉积分，位置预测
	virtual void predictPositions(Real dt);

	// 约束投影
	virtual void constraintsProjection(Real dt, d_uint iterations_number);

	// 更新速度
	virtual void updateVelocities(Real dt);


public:
	std::vector<EigenVector3> positions;
	std::vector<EigenVector3> positions_old;
	std::vector<EigenVector3> velocities;
	std::vector<EigenVector3> external_forces;
	std::vector<Real> masses;
	std::vector<Real> masses_inv;

	std::vector<PBD_Constraint*> constraints;
	// 碰撞约束
	std::vector<PBD_Constraint*> collision_constraints;

public:
	// orientation
	std::vector<Real> o_masses;
	std::vector<Real> o_masses_inv;
	std::vector<Quaternionr> o_q;
	std::vector<Quaternionr> o_q0;
	std::vector<Quaternionr> o_q_old;
	std::vector<EigenVector3> o_omega;
	std::vector<EigenVector3> o_alpha;
};


class PBD_ClothModel :public PBD_Model
{
public:
	/*
	* 初始化一个cloth（布料）二维模型
	*
	* 布料节点编号示意图：
	*	O-------------------------------------> x轴
	*	|	0	1	2	3	4
	*	|	5	6	7	8	9
	*	|	10	11	12	13	14
	*	|	15	16	17	18	19
	*	|
	*	|
	*	|
	*	|
	*	z轴
	*/
	PBD_ClothModel(
		Real width, Real height,	// 宽度，高度
		d_uint nCol, d_uint nRow,	// 节点列数，行数
		Real mass,					// 布料每个节点质量（每个节点质量相等）
		Real distanceStiffness, Real bendingStiffness
	);

	// 绘制
	virtual void draw();


private:
	// OpenGL
	d_uint VAO;
	d_uint VBO;
	d_uint EBO;
	std::vector<d_uint> indices;
};


enum class ThreadState
{
	in,
	out,
};

class PBD_ThreadModel :public PBD_Model
{
public:
	PBD_ThreadModel(
		Real length,	// 长度
		d_uint size,	// 节点数目
		Real mass,		// 每个节点质量
		EigenVector3 position,
		int constraintType
	);

	// 绘制
	virtual void draw(Shader& shader);

	virtual void step(Real dt, d_uint iterations_number, CollisionDetection& cd, PD_Simulation_Cuda& pdc);

	virtual void constraintsProjection(Real dt, d_uint iterations_number, bool is_Second = false);

	void dampVelocities();

	int which_path(int id);
	bool can_in(int vid);

private:
	// OpenGL
	d_uint VAO;
	d_uint VBO;
	d_uint EBO;
	std::vector<d_uint> indices;

public:
	bool is_collision_detection;
	bool is_collision_tet;

	// 缝合
	Real thread_ds;								// 缝合线初始节点间距
	EigenVector3 head_position;					// 缝合头节点位置
	Real suture_path_min_ds;					// 缝合路径最小间隔距离
	std::vector<ThreadState> thread_state;		// 记录缝合线节点状态
	//
	unsigned int path_number;
	std::vector<std::vector<EigenVector3>> suture_path;		// 记录缝合路径
	std::vector<std::vector<Real>> path_nodes_dis;			// 记录缝合路径节点间的距离
	std::vector<Real> path_length;
	bool is_end_reach;		// 记录缝合线倒数第二个节点是否进入软体
	//
	std::vector<bool> is_fixed;		// 记录缝合路径中的节点是否固定状态

	struct CTet
	{
		// x = a1p1 + a2p2 + a3p3 + a4p4
		// a1 + a2 + a3 + a4 = 1

		d_uint p1, p2, p3, p4;
		Real l1, l2, l3, l4;

		CTet(const Vector3r& x, d_uint tet_index, const CollisionDetection& cd);

		Vector3r update(const CollisionDetection& cd);
	};

	// 缝合路径节点与tet的联系
	std::vector<std::vector<CTet>> path_tet;


	// 为了保持第一段缝合线方向不变(useless)
	Quaternionr head_q;


	// 二次投影约束
	std::vector<Quaternionr> o_q_save;
	std::vector<EigenVector3> positions_save;
	void save();
	void recover();
};
