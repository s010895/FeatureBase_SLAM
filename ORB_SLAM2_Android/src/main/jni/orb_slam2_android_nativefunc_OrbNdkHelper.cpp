#include "orb_slam2_android_nativefunc_OrbNdkHelper.h"
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <errno.h>
#include <GLES2/gl2.h>
#include <android/asset_manager_jni.h>
#include<opencv2/core/core.hpp>
#include<System.h>
#include "GLS2Helper.h"
using namespace cv;
#include <android/log.h>
#define LOG_TAG "ORB_SLAM_SYSTEM"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
#define LOG(...) __android_log_print(ANDROID_LOG_ERROR,LOG_TAG, __VA_ARGS__)
static ORB_SLAM2::System *s;
static ORB_SLAM2::GLS2Helper *gls2Helper;
bool init_end = false;

/*
 * Class:     orb_slam2_android_nativefunc_OrbNdkHelper
 * Method:    initSystemWithParameters
 * Signature: (Ljava/lang/String;Ljava/lang/String;)V
 */
JNIEXPORT void JNICALL Java_orb_slam2_android_nativefunc_OrbNdkHelper_initSystemWithParameters
(JNIEnv * env, jclass cls, jstring VOCPath, jstring calibrationPath) {
	const char *calChar = env->GetStringUTFChars(calibrationPath, JNI_FALSE);
	const char *vocChar = env->GetStringUTFChars(VOCPath, JNI_FALSE);
	// use your string
	std::string voc_string(vocChar);
	std::string cal_string(calChar);
	env->GetJavaVM(&jvm);
	jvm->AttachCurrentThread(&env, NULL);

	s=new ORB_SLAM2::System(voc_string,cal_string,ORB_SLAM2::System::STEREO,true);
	env->ReleaseStringUTFChars(calibrationPath, calChar);
	env->ReleaseStringUTFChars(VOCPath, vocChar);
	init_end=true;
}
JNIEXPORT jintArray JNICALL Java_orb_slam2_android_nativefunc_OrbNdkHelper_startCurrentStereo(
		JNIEnv * env, jclass cls, jdouble curTimeStamp, jintArray buf,jintArray buf2, jint w,
		jint h) {
	jint *cbuf;
	jint *cbuf2;
	cbuf = env->GetIntArrayElements(buf, false);
	cbuf2 = env->GetIntArrayElements(buf2,false);
	if (cbuf == NULL) {
		return 0;
	}
	int size = w * h;
	cv::Mat myimg(h, w, CV_8UC4, (unsigned char*) cbuf);
	cv::Mat myimg2(h, w, CV_8UC4, (unsigned char*) cbuf2);
	//cv::Mat ima = s->TrackMonocular(myimg, curTimeStamp);
	LOG("Enter TrackStereo");
	cv::Mat ima = s->TrackStereo(myimg, myimg2,curTimeStamp);
	LOG("Out TrackStereo");
	jintArray resultArray = env->NewIntArray(ima.rows * ima.cols);
	jint *resultPtr;
	resultPtr = env->GetIntArrayElements(resultArray, false);
	for (int i = 0; i < ima.rows; i++)
		for (int j = 0; j < ima.cols; j++) {
			int R = ima.at < Vec3b > (i, j)[0];
			int G = ima.at < Vec3b > (i, j)[1];
			int B = ima.at < Vec3b > (i, j)[2];
			resultPtr[i * ima.cols + j] = 0xff000000 + (R << 16) + (G << 8) + B;
		}
	env->ReleaseIntArrayElements(resultArray, resultPtr, 0);
	env->ReleaseIntArrayElements(buf, cbuf, 0);
	env->ReleaseIntArrayElements(buf2, cbuf2, 0);
	return resultArray;
}
/*
 * Class:     orb_slam2_android_nativefunc_OrbNdkHelper
 * Method:    startCurrentORB
 * Signature: (DDD[I)[I
 */
JNIEXPORT jintArray JNICALL Java_orb_slam2_android_nativefunc_OrbNdkHelper_startCurrentORB(
		JNIEnv * env, jclass cls, jdouble curTimeStamp, jintArray buf, jint w,
		jint h) {
	jint *cbuf;
	cbuf = env->GetIntArrayElements(buf, false);
	if (cbuf == NULL) {
		return 0;
	}
	int size = w * h;
	cv::Mat myimg(h, w, CV_8UC4, (unsigned char*) cbuf);
	cv::Mat ima = s->TrackMonocular(myimg, curTimeStamp);
	jintArray resultArray = env->NewIntArray(ima.rows * ima.cols);
	jint *resultPtr;
	resultPtr = env->GetIntArrayElements(resultArray, false);
	for (int i = 0; i < ima.rows; i++)
		for (int j = 0; j < ima.cols; j++) {
			int R = ima.at < Vec3b > (i, j)[0];
			int G = ima.at < Vec3b > (i, j)[1];
			int B = ima.at < Vec3b > (i, j)[2];
			resultPtr[i * ima.cols + j] = 0xff000000 + (R << 16) + (G << 8) + B;
		}
	env->ReleaseIntArrayElements(resultArray, resultPtr, 0);
	env->ReleaseIntArrayElements(buf, cbuf, 0);
	return resultArray;
}
/*
 * Class:     orb_slam2_android_nativefunc_OrbNdkHelper
 * Method:    glesInit
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_orb_slam2_android_nativefunc_OrbNdkHelper_glesInit
(JNIEnv *env, jclass cls) {
    gls2Helper = new ORB_SLAM2::GLS2Helper();
    if(gls2Helper != nullptr){
        gls2Helper->create();
    }
	// 启用阴影平滑
	/*
	glShadeModel(GL_SMOOTH);
	// 黑色背景
	glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
	// 设置深度缓存
	glClearDepthf(1.0f);
	// 启用深度测试
	glEnable(GL_DEPTH_TEST);
	// 所作深度测试的类型
	glDepthFunc(GL_LEQUAL);
	// 告诉系统对透视进行修正
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
	*/
}

/*
 * Class:     orb_slam2_android_nativefunc_OrbNdkHelper
 * Method:    glesRender
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_orb_slam2_android_nativefunc_OrbNdkHelper_glesRender
(JNIEnv * env, jclass cls) {
    if(init_end)
    s->drawGL(gls2Helper);
    /*
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode (GL_MODELVIEW);
	glLoadIdentity ();
	if(init_end)
	s->drawGL();
	*/
}

/*
 * Class:     orb_slam2_android_nativefunc_OrbNdkHelper
 * Method:    glesResize
 * Signature: (II)V
 */
JNIEXPORT void JNICALL Java_orb_slam2_android_nativefunc_OrbNdkHelper_glesResize
(JNIEnv *env, jclass cls, jint width, jint height) {

    int changeWidth = width;
    int changeHeight = height;
    gls2Helper->change(width,height);
    /*
	//图形最终显示到屏幕的区域的位置、长和宽
	glViewport (0,0,width,height);
	//指定矩阵
	glMatrixMode (GL_PROJECTION);
	//将当前的矩阵设置为glMatrixMode指定的矩阵
	glLoadIdentity ();
	glOrthof(-2, 2, -2, 2, -2, 2);
	*/
}

/*
 * Class:     orb_slam2_android_nativefunc_OrbNdkHelper
 * Method:    readShaderFile
 * Signature: (Landroid/content/res/AssetManager;)V
 */
JNIEXPORT jintArray JNICALL Java_orb_slam2_android_nativefunc_OrbNdkHelper_startCurrentORBForCamera
(JNIEnv *env, jclass cls,jdouble timestamp, jlong addr,jint w,jint h) {
	const cv::Mat *im = (cv::Mat *) addr;
	cv::Mat ima = s->TrackMonocular(*im, timestamp);
	jintArray resultArray = env->NewIntArray(ima.rows * ima.cols);
	jint *resultPtr;
	resultPtr = env->GetIntArrayElements(resultArray, false);
	for (int i = 0; i < ima.rows; i++)
	for (int j = 0; j < ima.cols; j++) {
		int R = ima.at < Vec3b > (i, j)[0];
		int G = ima.at < Vec3b > (i, j)[1];
		int B = ima.at < Vec3b > (i, j)[2];
		resultPtr[i * ima.cols + j] = 0xff000000 + (R << 16) + (G << 8) + B;
	}
	env->ReleaseIntArrayElements(resultArray, resultPtr, 0);
	return resultArray;
}

