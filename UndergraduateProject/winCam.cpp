#include "winCam.h"

// 
glm::vec3 WinCam::cameraPosition;
glm::vec3 WinCam::cameraFront;
glm::vec3 WinCam::cameraUp;
glm::vec3 WinCam::cameraRight;
glm::vec3 WinCam::worldUp;
float WinCam::cameraYaw;		// 偏航角
float WinCam::cameraPitch;	// 俯仰角
float WinCam::cameraMoveSpeed;		// 相机移动速度
float WinCam::mouseSensitivity;		// 鼠标灵敏度
float WinCam::mouseZoom;					// 放缩
// mouse_callback 需要的一些参数
float WinCam::lastX;
float WinCam::lastY;
bool WinCam::firstMouse;


WinCam::WinCam(
	const int windowWidth,
	const int windowHeight,
	const std::string windowTitle,
	glm::vec3 cameraPosition,
	float cameraYaw,
	float cameraPitch
)
{
	firstMouse = true;
	lastFrame = glfwGetTime();
	deltaTime = 0;

	// 初始化glfw
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

	// 创建glfw窗口
	this->windowWidth = windowWidth;
	this->windowHeight = windowHeight;
	window = glfwCreateWindow(windowWidth, windowHeight, windowTitle.c_str(), NULL, NULL);
	if (window == NULL)
	{
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		return;
	}
	glfwMakeContextCurrent(window);
	// glad 加载所有opengl的函数指针
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		std::cout << "Failed to initialize GLAD" << std::endl;
		return;
	}

#ifdef CAMERA_USER_MOVE
	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

	// 设置回调函数
	glfwSetFramebufferSizeCallback(window, (GLFWframebuffersizefun)framebuffer_size_callback);
	glfwSetCursorPosCallback(window, mouse_callback);
	//glfwSetScrollCallback(window, scroll_callback);
#endif

	// 开启深度测试（Z-buffer)
	glEnable(GL_DEPTH_TEST);
	//glDepthFunc(GL_ALWAYS);
	//glDepthMask(GL_FALSE);


	// 初始化camera
	this->cameraFront = glm::vec3(0.0f, 0.0f, -1.0f);
	this->cameraMoveSpeed = 4.0f;
	this->mouseSensitivity = 0.1f;
	this->mouseZoom = 45.0f;
	this->cameraPosition = cameraPosition;
	this->worldUp = glm::vec3(0.0f, 1.0f, 0.0f);
	this->cameraYaw = cameraYaw;
	this->cameraPitch = cameraPitch;
	updateCameraVectors();
}

void WinCam::framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
	glViewport(0, 0, width, height);
}

void WinCam::mouse_callback(GLFWwindow* window, double xpos, double ypos)
{
	if (firstMouse)
	{
		lastX = xpos;
		lastY = ypos;
		firstMouse = false;
		return;
	}

	float xOffset = xpos - lastX;
	float yOffset = ypos - lastY;
	lastX = xpos;
	lastY = ypos;

	cameraYaw += xOffset * mouseSensitivity;
	cameraPitch -= yOffset * mouseSensitivity;
	if (cameraPitch > 89.0f) cameraPitch = 89.0f;
	if (cameraPitch < -89.0f) cameraPitch = -89.0f;

	// updateCameraVectors();  // 静态函数不能调用类中的非静态函数
	glm::vec3 front = glm::vec3(
		cos(glm::radians(cameraYaw)) * cos(glm::radians(cameraPitch)),
		sin(glm::radians(cameraPitch)),
		sin(glm::radians(cameraYaw)) * cos(glm::radians(cameraPitch))
	);
	cameraFront = glm::normalize(front);
	cameraRight = glm::normalize(glm::cross(cameraFront, worldUp));
	cameraUp = glm::normalize(glm::cross(cameraRight, cameraFront));
}


void WinCam::scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
	mouseZoom -= yoffset;
	if (mouseZoom < 1.0f) mouseZoom = 1.0f;
	if (mouseZoom > 45.0f) mouseZoom = 45.0f;
}

void WinCam::processInput()
{
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
	{
		glfwSetWindowShouldClose(window, true);
	}

#ifdef CAMERA_USER_MOVE
	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
		cameraPosition += cameraFront * cameraMoveSpeed * deltaTime;
	if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
		cameraPosition -= cameraFront * cameraMoveSpeed * deltaTime;
	if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
		cameraPosition -= cameraRight * cameraMoveSpeed * deltaTime;
	if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
		cameraPosition += cameraRight * cameraMoveSpeed * deltaTime;

	if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS)
		cameraPosition += cameraFront * cameraMoveSpeed * deltaTime * 0.1f;
	if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS)
		cameraPosition -= cameraFront * cameraMoveSpeed * deltaTime * 0.1f;
	if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS)
		cameraPosition -= cameraRight * cameraMoveSpeed * deltaTime * 0.1f;
	if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS)
		cameraPosition += cameraRight * cameraMoveSpeed * deltaTime * 0.1f;
#endif
}

void WinCam::updateDeltaTime()
{
	float currentFrame = glfwGetTime();
	deltaTime = currentFrame - lastFrame;
	lastFrame = currentFrame;
}

void WinCam::updateCameraMatrices()
{
	view = glm::lookAt(cameraPosition, cameraPosition + cameraFront, cameraUp);
	projection = glm::perspective(glm::radians((double)mouseZoom), 1.0 * windowWidth / windowHeight, 0.1, 100.0);
	model = glm::mat4(1.0f);
}

void WinCam::update()
{
	updateDeltaTime();
	updateCameraMatrices();
}

void WinCam::updateCameraVectors()
{
	glm::vec3 front = glm::vec3(
		cos(glm::radians(cameraYaw)) * cos(glm::radians(cameraPitch)),
		sin(glm::radians(cameraPitch)),
		sin(glm::radians(cameraYaw)) * cos(glm::radians(cameraPitch))
	);
	this->cameraFront = glm::normalize(front);
	this->cameraRight = glm::normalize(glm::cross(this->cameraFront, this->worldUp));
	this->cameraUp = glm::normalize(glm::cross(this->cameraRight, this->cameraFront));
}
