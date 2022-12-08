#pragma once

#include "../header.h"
#include "../load_model/tetMesh.h"
#include "pd_cuda.h"

/*
* ����ѧ������ı����϶���
*/


class PD_Simulation_Cuda
{
public:
	PD_Simulation_Cuda(std::string model_path, float step_time);

	void update();

	// init���ⲿ���ã������ʼ��ǰ����һЩ��Ķ���
	void init();

	void draw();

	// ��һ��������������ӹ̶���
	void addFixedPoint(glm::vec3 pos, float radius);
	// ��ĳ�������ĵ����Ϊ�̶���
	void addFixedPoint(int index);

	/*
	* λ��Լ����λ��Լ����ÿһ֡��̬�仯��
	* ÿ����Ҫ���ε������漸���������cuda��λ��Լ���ĸ���
	*/
	void resetPositionConstraint();									// ��������λ��Լ��
	void addPositionConstraint(d_uint index, const Vector3r& target);		// ���λ��Լ��
	void addPositionConstraint(const Vector3r& target, float radius);		// Ŀ��λ�����������ڵ����е����λ��Լ��
	void copyPositionConstraintToCuda();							// ����cuda����


	// ����λ����Ҫʵʱ���£���Ϊ��Ҫ��ײ���
	void updatePositionFromCUDA();


	// --------------- test ----------------------
	void addFixedPointTest()
	{
		int sum = 0;
		for (int i = 0; i < vertices_number; i++)
		{
			//if (positions[i * 3 + 0] < -0.8999 || positions[i * 3 + 0] > 0.8999)	// mesh.msh
			if (positions[i * 3 + 0] < -0.6399 || positions[i * 3 + 0] > 0.6399)	// mesh_narrow.msh
			//if (positions[i * 3 + 0] < -0.6599 || positions[i * 3 + 0] > 0.6599)	// mesh3.msh
			{
				addFixedPoint(i);
				sum++;
			}
		}
		printf("fixed points number: %d\n", sum);
	}
	// ------------------------------------------

private:
	void preMalloc();
	void load_model(std::string model_path);

	void initDraw();

public:
	// ģ�Ͳ���
	d_uint vertices_number;
	d_uint tet_number;
	d_int* tet_indices;				// ����������
	d_uint* tet_indices_for_draw;	// EBO����
	float* tet_stiffness;			// ÿ����������α�ϵ��
	float* positions;				// ����λ��
	float* mass;					// ��������
	// face
	unsigned int m_numFaces;
	unsigned int* m_face_indices;
	float* normal;

	// �������
	d_float dt;							// ֡ʱ����
	d_float gravity;					// ����
	// ����Լ��ǿ��
	d_float stiffness_position;			// λ��Լ��
	d_float stiffness_stretch;			// ���������Լ��
	d_float stiffness_bending;
	d_float damping_coefficient;		// �ٶ�����ϵ��
	d_uint iterations_per_frame;		// ��������
	float rho;							// �Ÿ�����������

	// ��������
	float* tetInvD3x3;			// �α����9m
	float* tetInvD3x4;			// ���ڼ���Խ���	12m
	float* tetVolume;			// ���������
	float* volumeDiag;			// w * Ac^T * Ac �ĶԽǷ���(w = k * vol)
	float* fixed;				// �Ƿ�̶���0��ʾ���̶���������ʾ�̶���

	// λ��Լ������
	std::vector<d_uint> position_constraint_indices;
	std::vector<float> position_constraint_targets;
	bool* add_position_constraint_index_flag;			// ȷ��ÿ���ڵ�ֻ���һ������Լ��������ȥ����


	// OpenGL
	d_uint VAO;
	d_uint VBO;
	d_uint NVBO;
	d_uint EBO;
};

