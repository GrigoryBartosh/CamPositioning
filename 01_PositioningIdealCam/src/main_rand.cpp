#include "my_algorythms.h"

#include <iostream>
#include <cmath>
#include <opencv2/opencv.hpp>
#include "CamModel.h"
#include "PositionCalculator.h"
#include "utils.h"
#include <libconfig.h++>

using namespace std;
using namespace Eigen;

using utils::vectorV2;
using utils::vectorV3;
using utils::PointPair23d;

using utils::pointsSRC2Pix;
using utils::pointsPix2Calibrated;
using utils::constructPointPairs;
using utils::point2Img;

using utils::readPointsJson;
using utils::writeTransformation;

static string FILE_POINTS_NAME;
static string FILE_TRANSFORM_NAME;
static string FILE_IMG_NAME;

static size_t CAM_W;
static size_t CAM_H;
static double CAM_ANGLE_X;

static void readConfig()
{
    libconfig::Config cfg;
    cfg.readFile("config/main_rand.cfg");

    FILE_POINTS_NAME = cfg.lookup("FILE_POINTS_NAME").c_str();
    FILE_TRANSFORM_NAME = cfg.lookup("FILE_TRANSFORM_NAME").c_str();
    FILE_IMG_NAME = cfg.lookup("FILE_IMG_NAME").c_str();

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

static void draw(const CamModel &cam, const vector<PointPair23d> &pointPairs, const vector<cv::Point2f> &pointsPix, const Affine3d &Transform)
{
    cout << Transform.matrix() << endl;
    cout << "error = " << PositionCalculator::getError(pointPairs, Transform) << endl;

    cv::Mat frame(CAM_H, CAM_W, CV_8UC3);

    for (const cv::Point2f &pix : pointsPix)
        cv::circle(frame, cv::Point(pix.x, pix.y), 3, cv::Scalar(0, 255, 0), -1);

    for (size_t i = 0; i < pointPairs.size(); i++)
    {
        const PointPair23d &point = pointPairs[i];
        cv::Point2f pix = point2Img(cam, Transform, point.world);
        cv::circle(frame, cv::Point(pix.x, pix.y), 3, cv::Scalar(0, 0, 255), -1);
    }

    imwrite(FILE_IMG_NAME, frame);
    cv::imshow("img", frame);
}

int my_algorythms::main_rand()
{
    init();

    CamModel cam(CAM_W, CAM_H);
    cam.setAngleX(CAM_ANGLE_X);

    vector<cv::Point2f> pointsPix;
    vectorV2 pointsCam;
    vectorV3 pointsWorld;
    vector<PointPair23d> pointPairs;
    Affine3d Transform = Affine3d::Identity();

    readPointsJson(FILE_POINTS_NAME, pointsWorld, pointsCam);
    pointsSRC2Pix(cam, pointsCam, pointsPix);

    pointsPix2Calibrated(cam, pointsPix, pointsCam);
    pointPairs = constructPointPairs(pointsWorld, pointsCam);
    Transform = PositionCalculator::getTransformationRand(pointPairs);
    
    writeTransformation(FILE_TRANSFORM_NAME, Transform);

    draw(cam, pointPairs, pointsPix, Transform);
    cv::waitKey();

    return 0;
}
