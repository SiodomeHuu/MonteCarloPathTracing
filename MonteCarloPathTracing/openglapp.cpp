#include "openglapp.h"
#include "config.h"
#include"OpenCLApp.h"

using namespace MCPT::Config;

namespace {
	HWND hWnd;
	GLuint glTex;
	int width;
	int height;


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
}


namespace MCPT::OpenGL {

	void init(int argc, char** argv) {
		width = WIDTH();
		height = HEIGHT();

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
	}

	void enterLoop() {
		glutMainLoop();
	}
	GLuint getTex() {
		return glTex;
	}

}