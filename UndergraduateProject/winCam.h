#pragma once

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <string>
#include <iostream>
#include <GLFW/glfw3.h>
#include "header.h"



/**
* �����������window�����camera���
* �����ֻ����Ϊ����ʹ�ã�����֮��Ҳͬʱ��ʼ����glfw
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
	static float cameraYaw;		// ƫ����
	static float cameraPitch;	// ������
	static float cameraMoveSpeed;		// ����ƶ��ٶ�
	static float mouseSensitivity;		// ���������
	static float mouseZoom;					// ����
	// mouse_callback ��Ҫ��һЩ����
	static float lastX;
	static float lastY;
	static bool firstMouse;
	// ʱ���¼
	float deltaTime = 0.0f;
	float lastFrame = 0.0f;
	// һЩ��ɫ��Ҫ�õ��ľ���
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

	// һЩcallback�����봦��
	static void framebuffer_size_callback(GLFWwindow* window, int width, int height);
	static void mouse_callback(GLFWwindow* window, double xpos, double ypos);
	static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
	void processInput();

	// ����ʱ��
	void updateDeltaTime();

	// ��������ľ�����ʹ����Щ����֮ǰһ��Ҫ����
	void updateCameraMatrices();

	// Ϊ�˷��㣬ÿһ֡����������º���
	void update();

private:
	void updateCameraVectors();
};
