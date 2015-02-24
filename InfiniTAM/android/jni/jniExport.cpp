#include "InfiniTAMApp.h"

#include <jni.h>

extern "C" {

JNIEXPORT void JNICALL Java_uk_ac_ox_robots_InfiniTAM_InfiniTAMRenderer_InitGL(JNIEnv *env, jobject thiz)
{
	(InfiniTAMApp::Instance())->InitGL();
}

JNIEXPORT void JNICALL Java_uk_ac_ox_robots_InfiniTAM_InfiniTAMRenderer_ResizeGL(JNIEnv *env, jobject thiz, int x, int y)
{
	(InfiniTAMApp::Instance())->ResizeGL(x,y);
}

JNIEXPORT void JNICALL Java_uk_ac_ox_robots_InfiniTAM_InfiniTAMRenderer_RenderGL(JNIEnv *env, jobject thiz)
{
	(InfiniTAMApp::Instance())->RenderGL();
}

JNIEXPORT void JNICALL Java_uk_ac_ox_robots_InfiniTAM_InfiniTAM_InitializeNativeApp(JNIEnv *env, jobject thiz)
{
	InfiniTAMApp::Instance();
}

JNIEXPORT int JNICALL Java_uk_ac_ox_robots_InfiniTAM_InfiniTAMProcessingThread_ProcessFrame(JNIEnv *env, jobject thiz)
{
	return (InfiniTAMApp::Instance())->ProcessFrame()?1:0;
}

}

