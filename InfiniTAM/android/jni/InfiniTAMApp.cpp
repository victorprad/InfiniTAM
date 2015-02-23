#include "InfiniTAMApp.h"

#include <GLES2/gl2.h>

InfiniTAMApp* InfiniTAMApp::globalInstance = NULL;//new InfiniTAMApp();

void InfiniTAMApp::InitGL(void)
{
	glClearColor(0.0f, 1.0f, 0.0f, 1.0f);
}

void InfiniTAMApp::ResizeGL(int newWidth, int newHeight)
{
	glViewport(0, 0, newWidth, newHeight);
}

void InfiniTAMApp::RenderGL(void)
{
	glClear(GL_COLOR_BUFFER_BIT);
}

