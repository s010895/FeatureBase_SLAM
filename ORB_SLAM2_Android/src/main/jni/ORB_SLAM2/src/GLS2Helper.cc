//
// Created by Smith on 2019/3/24.
//

#include "GLS2Helper.h"
#include "GLUtils.h"
#include <android/log.h>

#define LOG_TAG "ORB_SLAM_SYSTEM"
#define  LOGI(...)  __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)
#define  LOGD(...)  __android_log_print(ANDROID_LOG_DEBUG,LOG_TAG,__VA_ARGS__)
#define  LOGE(...)  __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,__VA_ARGS__)
namespace ORB_SLAM2{


static void printGLString(const char *name, GLenum s){
    const char *v = (const char *) glGetString(s);
    LOGI("GL %s = %s\n",name,v);
}

static void checkGLError(const char *op){
    for(GLint error=glGetError(); error; error=glGetError()){
        LOGI("after %s() glError (0x%x)\n",op,error);
    }
}

const char *VERTEX_SHADER=
        "uniform mat4 u_MVPMatrix;                 \n"
        "attribute vec4 a_Position;                \n"
        "attribute vec4 a_Color;                   \n"
        "varying vec4 v_Color;                     \n"
        "void main()                               \n"
        "{                                         \n"
        "   v_Color = a_Color;                      \n"
        "   gl_PointSize = 4.0;                     \n"
        "   gl_Position = u_MVPMatrix * a_Position; \n"
        "}                                          \n";

const char *FRAGMENT_SHADER=
        "precision mediump float; \n"
        "varying vec4 v_Color; \n"
        "void main()           \n"
        "{                      \n"
        "   gl_FragColor =  v_Color; \n"
        "}                        \n";
// This triangle is red, green, and blue.
GLfloat triangle1VerticesData[] = {
        // X, Y, Z,
        // R, G, B, A
        -0.5f, -0.25f, 1.5f,
        1.0f, 0.0f, 0.0f, 1.0f,

        0.5f, -0.25f, 1.5f,
        0.0f, 0.0f, 1.0f, 1.0f,

        0.0f, 0.559016994f, 1.5f,
        0.0f, 1.0f, 0.0f, 1.0f};
GLS2Helper::GLS2Helper(){
    mModelMatrix = NULL;
    mMVPMatrix = NULL;
    mProjectMatrix = NULL;
    mViewMatrix = NULL;
}

GLS2Helper::~GLS2Helper(){
    delete mModelMatrix;
    mModelMatrix = NULL;
    delete mMVPMatrix;
    mMVPMatrix = NULL;
    delete mProjectMatrix;
    mProjectMatrix = NULL;
    delete mViewMatrix;
    mViewMatrix = NULL;
}

void GLS2Helper::create(){
    printGLString("Version",GL_VERSION);
    printGLString("Vendor", GL_VENDOR);
    printGLString("Renderer", GL_RENDERER);
    printGLString("Extensions",GL_EXTENSIONS);

    mProgram = GLUtils::createProgram(&VERTEX_SHADER, &FRAGMENT_SHADER);
    if(!mProgram)
    {
        LOGD("Could not create program!!");
        return;
    }

    mModelMatrix = new Matrix();
    //mProjectMatrix = new Matrix();
    mMVPMatrix = new Matrix();
    float eyeX = 0.0f;
         float eyeY = 0.0f;
         float eyeZ = -0.02f;

            //We are looking at origin
         float centerX = 0.0f;
         float centerY = 0.0f;
         float centerZ = 0.0f;
            //Set our up vector
        float upX = 0.0f;
        float upY = 1.0f;
        float upZ = 0.0f;

        mViewMatrix = Matrix::newLookAt(eyeX, eyeY, eyeZ, centerX, centerY, centerZ, upX, upY, upZ);

}

void GLS2Helper::change(int width ,int height){
    glViewport(0, 0, width, height);

    float ratio = (float) width/height;
    float left = -ratio;
    float right = ratio;
    float bottom = -1.0f;
    float top = 1.0f;
    float near = 1.0f;
    float far = 10.0f;

    mProjectMatrix = Matrix::newFrustum(left, right, bottom, top, near,far);
    //mProjectMatrix->identity();
}
void GLS2Helper::setCameraPose(cv::Mat twc){

}
void GLS2Helper::draw(GLfloat *verticesData, size_t pointNumber,cv::Mat &mCameraPose, std::vector<float> &modelVector){

    //cv::Mat Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
    //cv::Mat twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
     //Position the eyes in front of origin
     /*
    float eyeX = twc.at<float>(0)*100;
    float eyeY = twc.at<float>(1)*100;
    float eyeZ = (twc.at<float>(2)*100)-2;

    //We are looking at origin
    float centerX = twc.at<float>(0)*100;
    float centerY = twc.at<float>(1)*100;
    float centerZ = twc.at<float>(2)*100;
    */

    glClearColor(0.0f,0.0f,0.0f,0.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    checkGLError("glClear");

    glUseProgram(mProgram);

    mMVPMatrixHandle = (GLuint) glGetUniformLocation(mProgram,"u_MVPMatrix");
    mPositionHandle = (GLuint) glGetAttribLocation(mProgram,"a_Position");
    mColorHandle = (GLuint) glGetAttribLocation(mProgram,"a_Color");

    mModelMatrix->identity();
    mModelMatrix->translate(modelVector[0], modelVector[1], modelVector[2]);
    glVertexAttribPointer(
    (GLuint) mPositionHandle,
    3,
    GL_FLOAT,
    GL_FALSE,
    4*7,
    verticesData);

    glEnableVertexAttribArray((GLuint)mPositionHandle);

    glVertexAttribPointer(
    (GLuint) mColorHandle,
    4,
    GL_FLOAT,
    GL_FALSE,
    4*7,
    verticesData+3);

    glEnableVertexAttribArray((GLuint)mColorHandle);

    mMVPMatrix->multiply(*mViewMatrix, *mModelMatrix);

    mMVPMatrix->multiply(*mProjectMatrix, *mMVPMatrix);

    glUniformMatrix4fv(mMVPMatrixHandle,1,GL_FALSE,mMVPMatrix->mData);
    glDrawArrays(GL_LINES,0,8);
    glDrawArrays(GL_POINTS,8,pointNumber);
    checkGLError("glDrawArrays");
    //delete mViewMatrix;

}

}