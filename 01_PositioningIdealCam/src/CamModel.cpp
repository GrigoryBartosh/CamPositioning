#include "CamModel.h"

using cv::Point2f;

using Eigen::Vector2d;

void CamModel::calcTrans()
{
    switch (significantAngle)
    {
    case ANGLE_X:
        trans = 2.0 * tan(angleW / 2) / camW;
        break;
    case ANGLE_Y:
        trans = 2.0 * tan(angleH / 2) / camH;
        break;
    }
}

CamModel::CamModel(size_t w, size_t h)
{
    setResolution(w, h);
}

void CamModel::setResolution(size_t w, size_t h)
{
    camW = w;
    camH = h;
    calcTrans();
}

void CamModel::setAngleX(double alpha)
{
    angleW = alpha;
    significantAngle = ANGLE_X;
    calcTrans();
}

void CamModel::setAngleY(double alpha)
{
    angleH = alpha;
    significantAngle = ANGLE_Y;
    calcTrans();
}

Point2f CamModel::getImgCoordinatesFromSRC(const Vector2d &v2) const
{
    double x = ( v2.x() / 2 + 0.5) * camW;
    double y = (-v2.y() / 2 + 0.5) * camH;
    Point2f p;
    p.x = x;
    p.y = y;

    return p;
}

Eigen::Vector2d CamModel::getSRCFromImgCoordinates(const cv::Point2f &pix) const
{
    double x = 2 * pix.x / camW - 1;
    double y = 1 - 2 * pix.y / camH;
    Vector2d v2;
    v2.x() = x;
    v2.y() = y;

    return v2;
}

Vector2d CamModel::getCalibratedCoordinates(const Point2f &pix) const
{
    double x = pix.x - camW / 2;
    double y = pix.y - camH / 2;
    Vector2d v2;
    v2.x() = x * trans;
    v2.y() = y * trans;

    return v2;
}

Point2f CamModel::getImgCoordinates(const Vector2d &v2) const
{
    double x = v2.x() / trans + camW / 2;
    double y = v2.y() / trans + camH / 2;
    Point2f p;
    p.x = x;
    p.y = y;

    return p;
}