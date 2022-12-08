#pragma once

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <string>
#include <iostream>
#include <GLFW/glfw3.h>
#include "header.h"



/**
* 整个类包括了window窗体和camera组件
* 这个类只能作为单例使用，建立之后也同时初始化了glfw
*/

class WinCam
{
public:
	// window
	GLFWwindow* window;
	int windowWidth;
	int windowHeight;
	// camera
	static glm::vec3 cameraPosition;
	static glm::vec3 cameraFront;
	static glm::vec3 cameraUp;
	static glm::vec3 cameraRight;
	static glm::vec3 worldUp;
	static float cameraYaw;		// 偏航角
	static float cameraPitch;	// 俯仰角
	static float cameraMoveSpeed;		// 相机移动速度
	static float mouseSensitivity;		// 鼠标灵敏度
	static float mouseZoom;					// 放缩
	// mouse_callback 需要的一些参数
	static float lastX;
	static float lastY;
	static bool firstMouse;
	// 时间记录
	float deltaTime = 0.0f;
	float lastFrame = 0.0f;
	// 一些着色器要用到的矩阵
	glm::mat4 model;
	glm::mat4 view;
	glm::mat4 projection;

	

	WinCam(
		const int windowWidth,
		const int windowHeight,
		const std::string windowTitle,
		glm::vec3 cameraPosition = glm::vec3(0.0f, 0.0f, 0.0f),
		float cameraYaw = -90.0f,
		float cameraPitch = 0.0f
	);

	// 一些callback和输入处理
	static void framebuffer_size_callback(GLFWwindow* window, int width, int height);
	static void mouse_callback(GLFWwindow* window, double xpos, double ypos);
	static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
	void processInput();

	// 更新时间
	void updateDeltaTime();

	// 更新相机的矩阵，在使用这些矩阵之前一定要更新
	void updateCameraMatrices();

	// 为了方便，每一帧调用这个更新函数
	void update();

private:
	void updateCameraVectors();
};
