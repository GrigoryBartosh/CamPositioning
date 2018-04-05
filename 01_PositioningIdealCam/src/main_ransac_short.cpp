#include "my_algorythms.h"

#include <iostream>
#include <cmath>
#include <opencv2/opencv.hpp>
#include "CamModel.h"
#include "PositionCalculator.h"
#include "PointPair23d.h"
#include "utils.h"

using namespace std;
using namespace Eigen;

using utils::vectorV2;
using utils::vectorV3;
using utils::getSubset;
using utils::constructPointPairs;
using utils::point2Img;

using utils::readPointsTxt;
using utils::writeTransformation;

static string FILE_POINTS_NAME;
static string FILE_TRANSFORM_NAME;

static size_t CAM_W;
static size_t CAM_H;
static double CAM_ANGLE_X;

static void readConfig()
{
    libconfig::Config cfg;
    cfg.readFile("config/main_ransac_short.cfg");

    FILE_POINTS_NAME = cfg.lookup("FILE_POINTS_NAME").c_str();
    FILE_TRANSFORM_NAME = cfg.lookup("FILE_TRANSFORM_NAME").c_str();

    cfg.readFile("config/camera.cfg");
    CAM_W = (int)cfg.lookup("CAM_W");
    CAM_H = (int)cfg.lookup("CAM_H");
    CAM_ANGLE_X = cfg.lookup("CAM_ANGLE_X");
    CAM_ANGLE_X *= M_PI / 180;
}

static void init()
{
    srand(time(NULL));

    readConfig();

    PositionCalculator::init();
}

static void draw(const CamModel &cam, const vector<PointPair23d> &pointPairs, const Affine3d &Transform)
{
    vector<size_t> inliners_num = PositionCalculator::getInlinersNum(pointPairs, Transform);
    vector<PointPair23d> inliners = getSubset(pointPairs, inliners_num);

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

int my_algorythms::main_ransac_short()
{
    init();

    CamModel cam(CAM_W, CAM_H);
    cam.setAngleX(CAM_ANGLE_X);

    vectorV2 pointsCam;
    vectorV3 pointsWorld;
    vector<PointPair23d> pointPairs;
    Affine3d Transform = Affine3d::Identity();

    readPointsTxt(FILE_POINTS_NAME, pointsWorld, pointsCam);
    pointPairs = constructPointPairs(pointsWorld, pointsCam);
    Transform = PositionCalculator::getTransformationRANSAC(pointPairs);
    
    writeTransformation(FILE_TRANSFORM_NAME, Transform);

    draw(cam, pointPairs, Transform);

    return 0;
}
