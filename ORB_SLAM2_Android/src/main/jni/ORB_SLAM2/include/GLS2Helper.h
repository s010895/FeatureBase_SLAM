//
// Created by Smith on 2019/3/24.
//

#ifndef ORB_SLAM2_ANDROID2_GLS2HELPER_H
#define ORB_SLAM2_ANDROID2_GLS2HELPER_H

#include<GLES2/gl2.h>
#include "Matrix.h"
#include<opencv2/core/core.hpp>
#include <vector>
namespace ORB_SLAM2{


class GLS2Helper{
public:
    GLS2Helper();

    ~GLS2Helper();

    void create();

    void change(int width, int height);

    void draw(GLfloat *verticesData, size_t pointNumber, GLfloat *cameraData, size_t cameraNumber,cv::Mat &mCameraPose, std::vector<float> &modelVector);

    void setCameraPose(cv::Mat twc);
private:
    float counter=0.0f;
    Matrix *mViewMatrix;
    Matrix *mModelMatrix;
    Matrix *mProjectMatrix;
    Matrix *mMVPMatrix;

    GLuint mProgram;
    GLuint mMVPMatrixHandle;
    GLuint mPositionHandle;
    GLuint mColorHandle;

    GLuint cameraProgram;
    GLuint cameraMatrixHandle;
    GLuint cameraPositionHandle;
    GLuint cameraColorHandle;
    cv::Mat cameraPose;



};


}
#endif //ORB_SLAM2_ANDROID2_GLS2HELPER_H
