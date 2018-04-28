#pragma once

#include <iostream>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>

class CamModel
{
private:
    enum SignificantAngle { ANGLE_X = 0, ANGLE_Y = 1 };

    size_t camW;
    size_t camH;

    SignificantAngle significantAngle = ANGLE_Y;
    double angleH = 40 * M_PI / 180; // 40 degree
    double angleW;

    double trans;

    void calcTrans();

public:
    CamModel(size_t w, size_t h);

    void setResolution(size_t w, size_t h);
    void setAngleX(double alpha);
    void setAngleY(double alpha);

    cv::Point2f getImgCoordinatesFromSRC(const Eigen::Vector2d &v2) const;
    Eigen::Vector2d getSRCFromImgCoordinates(const cv::Point2f &pix) const;
    Eigen::Vector2d getCalibratedCoordinates(const cv::Point2f &pix) const;
    cv::Point2f getImgCoordinates(const Eigen::Vector2d &v2) const;
};