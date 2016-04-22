// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "InfiniTAMApp.h"

#include <unistd.h>
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

JNIEXPORT void JNICALL Java_uk_ac_ox_robots_InfiniTAM_InfiniTAMApplication_InitializeNativeApp(JNIEnv *env, jobject thiz, jstring java_libdir)
{
	const char *native_libdir = env->GetStringUTFChars(/*env,*/ java_libdir, NULL);
	chdir(native_libdir);
	InfiniTAMApp::Instance();
	env->ReleaseStringUTFChars(/*env,*/ java_libdir, native_libdir);
}

JNIEXPORT void JNICALL Java_uk_ac_ox_robots_InfiniTAM_InfiniTAMProcessor_StartProcessing(JNIEnv *env, jobject thiz, int useLiveCamera)
{
	(InfiniTAMApp::Instance())->StartProcessing(useLiveCamera);
}

JNIEXPORT int JNICALL Java_uk_ac_ox_robots_InfiniTAM_InfiniTAMProcessor_ProcessFrame(JNIEnv *env, jobject thiz)
{
	return (InfiniTAMApp::Instance())->ProcessFrame()?1:0;
}

JNIEXPORT void JNICALL Java_uk_ac_ox_robots_InfiniTAM_InfiniTAMProcessor_StopProcessing(JNIEnv *env, jobject thiz)
{
	(InfiniTAMApp::Instance())->StopProcessing();
}

JNIEXPORT jfloat JNICALL Java_uk_ac_ox_robots_InfiniTAM_InfiniTAMProcessor_getAverageTime(JNIEnv *env, jobject thiz)
{
	return (InfiniTAMApp::Instance())->getAverageTime();
}

JNIEXPORT void JNICALL Java_uk_ac_ox_robots_InfiniTAM_InfiniTAMProcessor_toggleRecordingMode(JNIEnv *env, jobject thiz)
{
	(InfiniTAMApp::Instance())->toggleRecordingMode();
}

}

