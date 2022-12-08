#include "winCam.h"
#include "render/shader.h"
#include "render/texture.h"
#include "load_model/model.h"
#include "load_model/tetMesh.h"
#include "projective_dynamics/pd_simulation.h"
#include "projective_dynamics/pd_constraint.h"
#include "projective_dynamics/pd_simulation_cuda.h"

#include "position_based_dynamics/pbd_model.h"
#include "simulation/CollisionDetection.h"
using namespace std;


// MODE（三选一）
//#define OPERATE_MODE
//#define RECORD_MODE		// 此模式不要同时按两个或以上的按键，不然会出错
#define PLAY_MODE

int frame_tot = 0;
//


bool begin_simulation = false;
bool step_once = false;
bool print_debug = false;

float step_time = 0.01;

int vx, vy, vz;
float velocity = 0.003;

void input(GLFWwindow* window)
{
	if (glfwGetKey(window, GLFW_KEY_B) == GLFW_PRESS)
	{
		begin_simulation = !begin_simulation;
	}
	if (glfwGetKey(window, GLFW_KEY_O) == GLFW_PRESS)
	{
		step_once = true;
	}
	if (glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS)
	{
		print_debug = true;
	}

	if (glfwGetKey(window, GLFW_KEY_U) == GLFW_PRESS) vx = -1;
	else if (glfwGetKey(window, GLFW_KEY_I) == GLFW_PRESS) vx = 1;
	else vx = 0;

	if (glfwGetKey(window, GLFW_KEY_J) == GLFW_PRESS) vy = -1;
	else if (glfwGetKey(window, GLFW_KEY_K) == GLFW_PRESS) vy = 1;
	else vy = 0;

	if (glfwGetKey(window, GLFW_KEY_N) == GLFW_PRESS) vz = -1;
	else if (glfwGetKey(window, GLFW_KEY_M) == GLFW_PRESS) vz = 1;
	else vz = 0;
}

void input_record(GLFWwindow* window, std::vector<std::pair<int, int>>& record, int frame_iter)
{
	if (glfwGetKey(window, GLFW_KEY_B) == GLFW_PRESS)
	{
		begin_simulation = !begin_simulation;
		record.push_back(make_pair(frame_iter, GLFW_KEY_B));
	}
	if (glfwGetKey(window, GLFW_KEY_O) == GLFW_PRESS)
	{
		step_once = true;
		record.push_back(make_pair(frame_iter, GLFW_KEY_O));
	}
	if (glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS)
	{
		print_debug = true;
		record.push_back(make_pair(frame_iter, GLFW_KEY_P));
	}

	if (glfwGetKey(window, GLFW_KEY_U) == GLFW_PRESS)
	{
		vx = -1;
		record.push_back(make_pair(frame_iter, GLFW_KEY_U));
	}
	else if (glfwGetKey(window, GLFW_KEY_I) == GLFW_PRESS)
	{
		vx = 1;
		record.push_back(make_pair(frame_iter, GLFW_KEY_I));
	}
	else vx = 0;

	if (glfwGetKey(window, GLFW_KEY_J) == GLFW_PRESS)
	{
		vy = -1;
		record.push_back(make_pair(frame_iter, GLFW_KEY_J));
	}
	else if (glfwGetKey(window, GLFW_KEY_K) == GLFW_PRESS)
	{
		vy = 1;
		record.push_back(make_pair(frame_iter, GLFW_KEY_K));
	}
	else vy = 0;

	if (glfwGetKey(window, GLFW_KEY_N) == GLFW_PRESS)
	{
		vz = -1;
		record.push_back(make_pair(frame_iter, GLFW_KEY_N));
	}
	else if (glfwGetKey(window, GLFW_KEY_M) == GLFW_PRESS)
	{
		vz = 1;
		record.push_back(make_pair(frame_iter, GLFW_KEY_M));
	}
	else vz = 0;
}

void input_play(GLFWwindow* window, const std::vector<std::pair<int, int>>& record, int frame_iter, int& record_iter)
{
	if (record_iter >= record.size())
	{
		vx = 0;
		vy = 0;
		vz = 0;
		return;
	}
	std::pair<int, int> p = record[record_iter];
	int frame = p.first;
	int key = p.second;

	if (frame != frame_iter)
	{
		vx = 0;
		vy = 0;
		vz = 0;
		return;
	}

	if (GLFW_KEY_B == key)
	{
		begin_simulation = !begin_simulation;
	}
	if (GLFW_KEY_O == key)
	{
		step_once = true;
	}
	if (GLFW_KEY_P == key)
	{
		print_debug = true;
	}

	if (GLFW_KEY_U == key) vx = -1;
	else if (GLFW_KEY_I == key) vx = 1;
	else vx = 0;

	if (GLFW_KEY_J == key) vy = -1;
	else if (GLFW_KEY_K == key) vy = 1;
	else vy = 0;

	if (GLFW_KEY_N == key) vz = -1;
	else if (GLFW_KEY_M == key) vz = 1;
	else vz = 0;

	record_iter++;
}


int main()
{
	// winCam一定要最先创建，因为glfw的初始化在其中执行
	WinCam winCam(
		2400, 1500, "Undergraduate Project by Yinglong Li",
		glm::vec3(0.05, 1.38, 2.66),	// camera position
		-90.2,							// camera yaw
		-26.5							// camera pitch
	);
	//glfwSetKeyCallback(winCam.window, key_callback);
	glfwMaximizeWindow(winCam.window);

#ifdef DRAW_LINES
	Shader shader("../Resources/Shaders/tet.vs", "../Resources/Shaders/tet.fs");
#endif
#ifdef DRAW_SURFACE
	Shader shader("../Resources/Shaders/tet.vs", "../Resources/Shaders/tet.fs");
#endif
#ifdef DRAW_PHONG
	Shader shader("../Resources/Shaders/tet.vs", "../Resources/Shaders/tet_phong.fs");
	shader.use();
	// 材质
	shader.setVec3("material.ambient", glm::vec3(1, 1, 1));
	shader.setVec3("material.diffuse", glm::vec3(1, 1, 1));
	shader.setVec3("material.specular", glm::vec3(1, 1, 1));
	shader.setFloat("material.shininess", 32.0);
	// 方向光
	shader.setVec3("dirLight.direction", glm::vec3(-1, -1, 0));
	shader.setVec3("dirLight.ambient", glm::vec3(0.5, 0.5, 0.5));
	shader.setVec3("dirLight.diffuse", glm::vec3(0.5, 0.5, 0.5));
	shader.setVec3("dirLight.specular", glm::vec3(0.5, 0.5, 0.5));
#endif

	Shader shader2("../Resources/Shaders/thread.vs", "../Resources/Shaders/thread.fs");
	Shader shader22("../Resources/Shaders/thread.vs", "../Resources/Shaders/thread2.fs");

	// 缝合线
	PBD_ThreadModel thread(1.5, 75, 0.5, EigenVector3(0.4, 0.6, 0), 0);

	// 软体模型
	PD_Simulation_Cuda softbody("../Resources/Models/mesh/mesh_narrow.msh", step_time);
	softbody.addFixedPointTest();
	softbody.init();

	// 碰撞检测
	CollisionDetection cd;
	cd.initTetBSH(
		(const Vector3r*)softbody.positions,
		softbody.vertices_number,
		(const d_uint*)softbody.tet_indices,
		softbody.tet_number,
		(const d_uint*)softbody.m_face_indices,
		softbody.m_numFaces,
		0);

	// FPS
	double totTime = 0;
	int cnt = 0;


	// MODE
#ifdef RECORD_MODE
	puts("RECORD MODE");
	std::vector<std::pair<int, int>> record;
	FILE* p;
	fopen_s(&p, "record/rec2.txt", "w");
#endif

#ifdef PLAY_MODE
	puts("PLAY MODE");
	std::vector<std::pair<int, int>> record;
	int record_iter = 0;
	FILE* p;
	fopen_s(&p, "record/rec2.txt", "r");
	int iter, key;
	while (~fscanf_s(p, "%d %d", &iter, &key))
	{
		record.push_back(make_pair(iter, key));
	}
	fclose(p);
	printf("%d\n", record.size());
#endif


	while (!glfwWindowShouldClose(winCam.window))
	{
		// 交换缓冲
		glfwPollEvents();
		glfwSwapBuffers(winCam.window);

		// 必须调用winCam的每一帧更新函数
		winCam.update();
		//cout << "winCam.dt: " << winCam.deltaTime << "\t\tFPS: " << 1.0 / winCam.deltaTime << endl;
		//cout << frame_tot << endl;

		// 处理输入
		winCam.processInput();

#ifdef RECORD_MODE
		input_record(winCam.window, record, frame_tot);
#endif
#ifdef PLAY_MODE
		input_play(winCam.window, record, frame_tot, record_iter);
#endif
#ifdef OPERATE_MODE
		input(winCam.window);
#endif


		// 清理window
		glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


		// 主要循环代码
		shader.use();
		shader.setMat4("projection", winCam.projection);
		shader.setMat4("view", winCam.view);
		shader.setMat4("model", winCam.model);
#ifdef DRAW_PHONG
		shader.setVec3("viewPos", winCam.cameraPosition);
#endif

		shader2.use();
		shader2.setMat4("projection", winCam.projection);
		shader2.setMat4("view", winCam.view);
		shader2.setMat4("model", winCam.model);

		// 仿真

		if (begin_simulation == true || step_once == true)
		{
			// 软体仿真，更新位置
			softbody.update();
			softbody.updatePositionFromCUDA();

			// 软体的BSH更新
			cd.update();


			thread.head_position.x() += velocity * vx;
			thread.head_position.y() += velocity * vy;
			thread.head_position.z() += velocity * vz;
			thread.step(step_time, 20, cd, softbody);

			step_once = false;
		}

		shader.use();
		softbody.draw();
		shader2.use();
		thread.draw(shader2);

		if (print_debug)
		{
			print_debug = false;

			puts("++++++++++++++++++++++++++++++++++++++");

			printf("%d\n", cd.m_bshTet.m_numTets);

			BoundingSphere bs = cd.m_bshTet.hull(0);
			printf("%f %f %f:  %f\n", bs.x().x(), bs.x().y(), bs.x().z(), bs.r());

			puts("--------------------------------------\n");
		}

		// 按照帧率运行
		while (glfwGetTime() - winCam.lastFrame < step_time);

		if (begin_simulation)
		{
			totTime += winCam.deltaTime;
			cnt++;
		}

		frame_tot++;
	}

	// 结束
	glfwTerminate();

	printf("average dt: %f,  average fps: %f\n", totTime / cnt, (double)cnt / totTime);


#ifdef RECORD_MODE
	puts("RECORD operation sequence");
	for (auto s : record)
	{
		fprintf(p, "%d %d\n", s.first, s.second);
}
	fclose(p);
	printf("%d\n", record.size());
#endif

	return 0;
}
