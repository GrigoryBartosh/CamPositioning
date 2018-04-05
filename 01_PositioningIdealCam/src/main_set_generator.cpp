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
using utils::randomAffine3d;
using utils::randomFromTo;
using utils::pointsSRC2Pix;
using utils::pointsPix2Calibrated;

using utils::writePointsJson;
using utils::writeTransformation;

static string FILE_POINTS_NAME;
static string FILE_TRANSFORM_NAME;

static size_t POINTS_COUNT;
static double LEN_FROM;
static double LEN_TO;

static size_t CAM_W;
static size_t CAM_H;
static double CAM_ANGLE_X;

static void readConfig()
{
    libconfig::Config cfg;
    cfg.readFile("config/main_set_generator.cfg");

    FILE_POINTS_NAME = cfg.lookup("FILE_POINTS_NAME").c_str();
    FILE_TRANSFORM_NAME = cfg.lookup("FILE_TRANSFORM_NAME").c_str();
    POINTS_COUNT = (int)cfg.lookup("POINTS_COUNT");
    LEN_FROM = cfg.lookup("LEN_FROM");
    LEN_TO = cfg.lookup("LEN_TO");

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

static void generateSRC(vectorV2 &pointsSrc)
{
    for (size_t i = 0; i < POINTS_COUNT; i++)
    {
        Vector2d v2;
        v2 << randomFromTo(-1, 1), randomFromTo(-1, 1);
        pointsSrc.push_back(v2);
    }
}

static void generateWorld(const Affine3d &Transform, const vectorV2 &pointsCam, vectorV3 &pointsWorld)
{
    Affine3d TInv = Transform.inverse();
    for (const Vector2d &v2 : pointsCam)
    {
        Vector3d v3;
        v3 << v2.x(), v2.y(), 1;
        v3 *= randomFromTo(LEN_FROM, LEN_TO);
        v3 = TInv * v3;
        pointsWorld.push_back(v3);
    }
}

static void draw(const Affine3d &Transform, const vector<cv::Point2f> &pointsPix)
{
    cout << Transform.matrix() << endl;

    cv::Mat frame(CAM_H, CAM_W, CV_8UC3);

    for (const cv::Point2f &pix : pointsPix)
        cv::circle(frame, cv::Point(pix.x, pix.y), 3, cv::Scalar(0, 255, 0), -1);

    imwrite("img.jpg", frame);
    cv::imshow("img", frame);
    cv::waitKey();
}

int my_algorythms::main_set_generator()
{
    init();

    CamModel cam(CAM_W, CAM_H);
    cam.setAngleX(CAM_ANGLE_X);

    Affine3d Transform = Affine3d::Identity();
    vectorV2 pointsSrc;
    vector<cv::Point2f> pointsPix;
    vectorV2 pointsCam;
    vectorV3 pointsWorld;

    generateSRC(pointsSrc);
    pointsSRC2Pix(cam, pointsSrc, pointsPix);
    pointsPix2Calibrated(cam, pointsPix, pointsCam);

    Transform = randomAffine3d();
    generateWorld(Transform, pointsCam, pointsWorld);

    writePointsJson(FILE_POINTS_NAME, pointsWorld, pointsSrc);    
    writeTransformation(FILE_TRANSFORM_NAME, Transform);

    draw(Transform, pointsPix);

    return 0;
}
