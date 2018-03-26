#include <iostream>
#include <cstdlib>
#include "CamModel.h"
#include "PositionCalculator.h"
#include "PointPair23d.h"

using namespace std;
using namespace Eigen;

const string FILE_POINTS_NAME = "points.txt";
const string FILE_TRANSFORM_NAME = "transformation.txt";

const size_t CAM_W = 1280;
const size_t CAM_H = 720;
const double CAM_ANGLE_X = 60 * M_PI / 180;

typedef vector<Vector2d, aligned_allocator<Vector2d> > vectorV2;
typedef vector<Vector3d, aligned_allocator<Vector3d> > vectorV3;

void readPoints(vectorV3 &pointsWorld, vectorV2 &pointsCam)
{
    ifstream in(FILE_POINTS_NAME);
    if (!in.is_open())
    {
        cerr << "Could not open file '" + FILE_POINTS_NAME + "'";
        exit(1);
    }

    double t;
    while (in >> t)
    {
        Vector3d v3;
        Vector2d v2;

        v3(0) = t;
        in >> t;
        v3(1) = t;
        in >> t;
        v3(2) = t;

        in >> t;
        v2(0) = t;
        in >> t;
        v2(1) = t;

        pointsWorld.push_back(v3);
        pointsCam.push_back(v2);
    }

    cout << pointsWorld.size() << endl;

    in.close();
}

void writeTransformation(const Affine3d &Transform)
{
    ofstream out(FILE_TRANSFORM_NAME);
    if (!out.is_open())
    {
        cerr << "Could not open file '" + FILE_TRANSFORM_NAME + "'";
        exit(1);
    }

    out << Transform.matrix();

    out.close();
}

void pointsSRC2Pix(const CamModel &cam, const vectorV2 &pointsCam, vector<cv::Point2f> &pointsPix)
{
    pointsPix.clear();
    for (const Vector2d &v2 : pointsCam)
        pointsPix.push_back(cam.getImgCoordinatesFromSRC(v2));
}

void pointsPix2Calibrated(const CamModel &cam, const vector<cv::Point2f> &pointsPix, vectorV2 &pointsCam)
{
    pointsCam.clear();
    for (const cv::Point2f &pix : pointsPix)
        pointsCam.push_back(cam.getCalibratedCoordinates(pix));
}

vector<PointPair23d> constructPointPirs(const vectorV3 &pointsWorld, const vectorV2 &pointsCam)
{
    vector<PointPair23d> pointPairs;

    size_t n = pointsWorld.size();
    for (size_t i = 0; i < n; i++)
        pointPairs.push_back(PointPair23d(pointsWorld[i], pointsCam[i]));

    return pointPairs;
}

cv::Point2f point2Img(const CamModel &cam, const Affine3d &T, Vector3d v3)
{
    v3 = T * v3;
    Vector2d v2 = PositionCalculator::project(v3);
    cv::Point2f p = cam.getImgCoordinates(v2);
    return p;
}

void draw(const CamModel &cam, const vector<PointPair23d> &pointPairs, const Affine3d &Transform)
{
    vector<size_t> inliners_num = PositionCalculator::getInlinersNum(pointPairs, Transform);
    vector<PointPair23d> inliners = PositionCalculator::getSubset(pointPairs, inliners_num);

    cout << Transform.matrix() << endl;
    cout << "error = " << PositionCalculator::getError(inliners, Transform) << endl;

    cv::Mat frame(CAM_H, CAM_W, CV_8UC3);

    for (size_t i = 0, j = 0; i < pointPairs.size(); i++)
    {
        cv::Scalar color;
        if (j < inliners_num.size() && i == inliners_num[j]) 
        {
            color = cv::Scalar(0, 0, 255);
            j++;
        } else {
            color = cv::Scalar(255, 0, 0);
        }

        const PointPair23d &point = pointPairs[i];
        cv::Point2f pix = cam.getImgCoordinates(point.cam);
        cv::circle(frame, cv::Point(pix.x, pix.y), 3, cv::Scalar(0, 255, 0), -1);

        pix = point2Img(cam, Transform, point.world);
        cv::circle(frame, cv::Point(pix.x, pix.y), 3, color, -1);
    }

    imwrite("img.jpg", frame);
    cv::imshow("img", frame);
    cv::waitKey();
}

int main()
{
    srand(time(NULL));

    CamModel cam(CAM_W, CAM_H);
    cam.setAngleX(CAM_ANGLE_X);

    PositionCalculator::init();

    vectorV2 pointsCam;
    vectorV3 pointsWorld;
    vector<PointPair23d> pointPairs;
    Affine3d Transform = Affine3d::Identity();

    readPoints(pointsWorld, pointsCam);
    pointPairs = constructPointPirs(pointsWorld, pointsCam);
    Transform = PositionCalculator::getTransformation(pointPairs);
    
    writeTransformation(Transform);

    draw(cam, pointPairs, Transform);

    return 0;
}
