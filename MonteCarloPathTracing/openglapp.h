#pragma once
#include<GL/glew.h>
#include<GL/glut.h>


namespace MCPT::OpenGL {

	void init(int argc,char** argv);


	void enterLoop();

	GLuint getTex();
}