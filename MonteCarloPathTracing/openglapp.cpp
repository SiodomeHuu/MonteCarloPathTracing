#include "openglapp.h"
#include "config.h"
#include"OpenCLApp.h"

#include "objdef.h"
#include "auxiliary.h"
#include "raygeneration.h"
#include "colorout.h"

#include <iostream>

using namespace MCPT::Config;
using namespace MCPT;

namespace {
	HWND hWnd;
	GLuint glTex;
	int width;
	int height;
	
	MCPT::Camera camera;
	bool needUpdate = false;

	void displayFunc() {
		glEnable(GL_TEXTURE_2D);
		glEnable(GL_TEXTURE_RECTANGLE);
		glClear(GL_COLOR_BUFFER_BIT);
		glBindTexture(GL_TEXTURE_2D, glTex);
		
		glBegin(GL_QUADS);
		glTexCoord2f(0, 0);
		glVertex2f(-1.0f, -1.0f);

		glTexCoord2f(0, HEIGHT());
		glVertex2f(-1.0f, 1.0f);
		glTexCoord2f(width, height);
		glVertex2f(1.0f, 1.0f);
		glTexCoord2f(width, 0);
		glVertex2f(1.0f, -1.0f);
		glEnd();

		glBindTexture(GL_TEXTURE_2D, NULL);
		glDisable(GL_TEXTURE_2D);
		glDisable(GL_TEXTURE_RECTANGLE);

		glFinish();
		glutSwapBuffers();
	}

	void idleFunc() {

		static float framesPerSecond = 0.0f;
		static float lastTime = 0.0f;
		static std::wstring str;
		static float currentTime;
		
		currentTime = GetTickCount64() * 0.001f;

		if (needUpdate) {
			auto cmrToWrite = camera;
			cmrToWrite.up = normalize(cross(camera.horizontal, camera.direction));
			MCPT::RayGeneration::resetCamera(cmrToWrite);
			MCPT::ColorOut::refresh();
		}
		
		MCPT::OpenCL::update();

		++framesPerSecond;
		if (currentTime - lastTime > 1.0f) {
			lastTime = currentTime;
			str = L"Monte Carlo Path Tracer FPS: " + std::to_wstring(framesPerSecond);
			SetWindowTextW(hWnd, str.c_str());
			framesPerSecond = 0;
		}
	}

	void OnTimer(int value) {
		glutPostRedisplay();
		glutTimerFunc(10, OnTimer, 1);
	}

	
	
	void processKey(unsigned char key, int x, int y) {
		static float step = 5.0f;
		if (Config::TESTBVH()) {
			switch (key) {
				case 'w':
				{
					camera.center = camera.center + (step * camera.direction);
					needUpdate = true;
				}
				break;
				case 's':
				{
					camera.center = camera.center - (step * camera.direction);
					needUpdate = true;
				}
				break;
				case 'a':
				{
					cl_float3 left = normalize(cross(camera.up, camera.direction));
					camera.center = camera.center + step * left;
					needUpdate = true;
				}
				break;
				case 'd':
				{
					cl_float3 left = normalize(cross(camera.up, camera.direction));
					camera.center = camera.center - step * left;
					needUpdate = true;
				}
				break;
				case 'q':
				{
					exit(0);
				}
				break;
				case 'e':
				{
					std::cout << "{ \"position\":[";
					std::cout << camera.center.x << "," << camera.center.y << "," << camera.center.z << "],";
					std::cout << "\"lookat\": [";
					auto lookat = camera.center + camera.direction;
					std::cout << lookat.x << "," << lookat.y << "," << lookat.z << "],";
					std::cout << "\"up\": [";
					std::cout << camera.up.x << "," << camera.up.y << "," << camera.up.z << "],";
					std::cout << "\"fov\": 45, \"resolution\":[1280,720] },"<<std::endl;
				}
				break;
				case 'r': {
					glutWarpPointer(width / 2, height / 2);
				}
				break;
				case '1': {
					step = 0.1f;
				}
				break;
				case '2': {
					step = 0.5f;
				}
				break;
				case '3': {
					step = 5.0f;
				}
				break;
				case '4': {
					step = 10.0f;
				}break;
				case '5': {
					step = 100.0f;
				}break;
				case '6': {
					step = 500.0f;
				}break;
				case '7': {
					step = 10000.0f;
				}
				break;
			}
		}
	}

	void mouseCB(int x, int y) {
		static const float step = 100.0;
		static auto initer = [&]() {
			glutWarpPointer(width / 2, height / 2);
			return 0;
		}();

		int deltaX = x - width / 2;
		int deltaY = -y +  height / 2;

		float radX = deltaX / step;
		float radY = deltaY / step;

		float4 newDirection = cos(radX) * camera.direction + sin(radX) * camera.horizontal;
		float4 newHorizontal = normalize(cross(newDirection, camera.up));
		newDirection = cos(radY) * newDirection + sin(radY) * camera.up;
		//float4 newUp = normalize(cross(newHorizontal, newDirection));

		camera.direction = normalize(newDirection);
		//camera.up = newUp;
		camera.horizontal = newHorizontal;

		needUpdate = true;

		glutWarpPointer(width / 2, height / 2);
	}
}


namespace MCPT::OpenGL {

	void init(int argc, char** argv) {
		width = WIDTH();
		height = HEIGHT();
		//camera = MCPT::Auxiliary::parseCamera(Config::GETCAMERA());
		{
			auto jsonCamera = Config::GETCAMERA();
			auto jsPos = jsonCamera["position"].get<std::vector<json> >();
			auto jsLookat = jsonCamera["lookat"].get<std::vector<json> >();
			auto jsUp = jsonCamera["up"].get<std::vector<json> >();

			for (int i = 0; i < 3; ++i) {
				camera.center.s[i] = float(jsPos[i]);
			}

			float4 lookat;
			for (int i = 0; i < 3; ++i) {
				lookat.s[i] = float(jsLookat[i]);
			}
			camera.direction = lookat - camera.center;
			camera.direction.w = 0.0f;

			for (int i = 0; i < 3; ++i) {
				camera.up.s[i] = float(jsUp[i]);
			}
			camera.arg = float(jsonCamera["fov"]) * M_PI / 180.0f;

			float4 horizontal = cross(camera.direction, camera.up);
			camera.tmin = 0;
			camera.direction = normalize(camera.direction);
			camera.up = normalize(camera.up);
			camera.horizontal = normalize(horizontal);
		}


		glutInit(&argc, argv);
		glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
		glutInitWindowSize(width, height);
		glutCreateWindow("PathTracer");

		hWnd = GetActiveWindow();


		glGenTextures(1, &glTex);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, glTex);
		glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
		glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, GL_RGBA32F_ARB, width, height, 0, GL_LUMINANCE, GL_FLOAT, NULL);


		glutDisplayFunc(displayFunc);
		glutIdleFunc(idleFunc);
		glutTimerFunc(10, OnTimer, 1);

		glutKeyboardFunc(processKey);
		glutMotionFunc(mouseCB);
	}

	void enterLoop() {
		glutMainLoop();
	}
	GLuint getTex() {
		return glTex;
	}

}