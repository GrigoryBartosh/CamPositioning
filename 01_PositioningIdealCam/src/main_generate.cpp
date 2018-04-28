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
using utils::Sample;

using utils::checkInput;
using utils::unknownComand;

using utils::randomFromTo;
using utils::pointsSRC2Pix;
using utils::pointsPix2Calibrated;
using utils::randomAffine3d;
using utils::randomGaussian;
using utils::pointsPix2SRC;
using utils::constructPointPairs;

using utils::writePointsJson;
using utils::writeTransformation;
using utils::openToWrite;
using utils::writeSample;
using utils::writeGroup;

static string FILE_POINTS_NAME;
static string FILE_TRANSFORM_NAME;
static string FILE_SAMPLES_SINGLE_NAME;
static string FILE_SAMPLES_GROUP_NAME;

static size_t SAMPLES_COUNT;
static size_t GROUPS_COUNT;
static size_t GROUP_SIZE;

static size_t POINTS_COUNT_FROM;
static size_t POINTS_COUNT_TO;
static double LEN_FROM;
static double LEN_TO;
static double WHITE_NOISE_RADIUS;

static size_t CAM_W;
static size_t CAM_H;
static double CAM_ANGLE_X;

static void readConfig()
{
    libconfig::Config cfg;
    cfg.readFile("config/main_generate.cfg");

    FILE_POINTS_NAME = cfg.lookup("FILE_POINTS_NAME").c_str();
    FILE_TRANSFORM_NAME = cfg.lookup("FILE_TRANSFORM_NAME").c_str();
    FILE_SAMPLES_SINGLE_NAME = cfg.lookup("FILE_SAMPLES_SINGLE_NAME").c_str();
    FILE_SAMPLES_GROUP_NAME = cfg.lookup("FILE_SAMPLES_GROUP_NAME").c_str();

    SAMPLES_COUNT = (int)cfg.lookup("SAMPLES_COUNT");
    GROUPS_COUNT = (int)cfg.lookup("GROUPS_COUNT");
    GROUP_SIZE = (int)cfg.lookup("GROUP_SIZE");

    POINTS_COUNT_FROM = (int)cfg.lookup("POINTS_COUNT_FROM");
    POINTS_COUNT_TO = (int)cfg.lookup("POINTS_COUNT_TO");
    LEN_FROM = cfg.lookup("LEN_FROM");
    LEN_TO = cfg.lookup("LEN_TO");
    WHITE_NOISE_RADIUS = cfg.lookup("WHITE_NOISE_RADIUS");

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

void generateSRC(vectorV2 &pointsSrc, size_t pointsCount)
{
    pointsSrc.clear();
    for (size_t i = 0; i < pointsCount; i++)
    {
        Vector2d v2;
        v2 << randomFromTo(-1, 1), randomFromTo(-1, 1);
        pointsSrc.push_back(v2);
    }
}

void generateWorld(const Affine3d &Transform, const vectorV2 &pointsCam, vectorV3 &pointsWorld)
{
    pointsWorld.clear();
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

void addWhiteNoise(vector<cv::Point2f> &pointsPix, double radius)
{
    for (cv::Point2f &pix : pointsPix)
    {
        pix.x += randomGaussian(radius);
        pix.y -= randomGaussian(radius);
    }
}

static void makeSet(const CamModel &cam, vectorV2 &pointsSrc, Affine3d &Transform, vectorV3 &pointsWorld)
{
    vector<cv::Point2f> pointsPix;
    vectorV2 pointsCam;

    size_t points_count = round(randomFromTo(POINTS_COUNT_FROM, POINTS_COUNT_TO));
    generateSRC(pointsSrc, points_count);
    pointsSRC2Pix(cam, pointsSrc, pointsPix);
    pointsPix2Calibrated(cam, pointsPix, pointsCam);

    Transform = randomAffine3d();
    generateWorld(Transform, pointsCam, pointsWorld);

    addWhiteNoise(pointsPix, WHITE_NOISE_RADIUS);
    pointsPix2SRC(cam, pointsPix, pointsSrc);
}

static Sample makeSample(const CamModel &cam, const Affine3d &srcTransform, const vectorV2 &pointsSrc, const vectorV3 &pointsWorld)
{
    vector<cv::Point2f> pointsPix;
    vectorV2 pointsCam;
    vector<PointPair23d> pointPairs;

    pointsSRC2Pix(cam, pointsSrc, pointsPix);
    pointsPix2Calibrated(cam, pointsPix, pointsCam);

    Sample sample;
    pointPairs = constructPointPairs(pointsWorld, pointsCam);
    Affine3d Transform = PositionCalculator::getTransformationStep(pointPairs, NULL, &sample.shifts, NULL);

    Vector3d camPos = srcTransform * Transform.inverse() * Vector3d(0, 0, 0);
    sample.camX = camPos.x();

    sample.err = PositionCalculator::getError(pointPairs, Transform);
    return sample;
}

static void draw(const Affine3d &Transform)
{
    cout << Transform.matrix() << endl;
}

int my_algorythms::main_generate(int argc, char* argv[])
{
    if (checkInput(argc)) return 1;

    init();

    const char* type = argv[1];
    --argc;
    ++argv;

    CamModel cam(CAM_W, CAM_H);
    cam.setAngleX(CAM_ANGLE_X);

    vectorV2 pointsSrc;
    vectorV3 pointsWorld;
    Affine3d Transform;

    if (strcmp(type, "set") == 0)
    {
        makeSet(cam, pointsSrc, Transform, pointsWorld);

        writePointsJson(FILE_POINTS_NAME, pointsWorld, pointsSrc);    
        writeTransformation(FILE_TRANSFORM_NAME, Transform);

        draw(Transform);
    }
    else if (strcmp(type, "samples") == 0) 
    {
        if (checkInput(argc)) return 1;

        type = argv[1];

        if (strcmp(type, "single") == 0)
        {
            ofstream os = openToWrite(FILE_SAMPLES_SINGLE_NAME, SAMPLES_COUNT);

            for (size_t i = 0; i < SAMPLES_COUNT; i++)
            {
                if (i % 1000 == 0) cout << "Done " << i / 1000 << " iterations of " << SAMPLES_COUNT / 1000 << " (x1000)" << endl;

                makeSet(cam, pointsSrc, Transform, pointsWorld);
                Sample s = makeSample(cam, Transform, pointsSrc, pointsWorld);

                writeSample(os, s);
            }

            os.close();
        }
        else if (strcmp(type, "group") == 0)
        {
            ofstream os = openToWrite(FILE_SAMPLES_GROUP_NAME, GROUPS_COUNT);

            for (size_t i = 0; i < GROUPS_COUNT; i++)
            {
                cout << "Done " << i << " iterations of " << GROUPS_COUNT << endl;

                vector<Sample> samples;

                makeSet(cam, pointsSrc, Transform, pointsWorld);
                for (size_t j = 0; j < GROUP_SIZE; j++)
                    samples.push_back(makeSample(cam, Transform, pointsSrc, pointsWorld));

                writeGroup(os, samples);
            }

            os.close();
        }
        else
        {
            return unknownComand();
        }
    }
    else 
    {
        return unknownComand();
    }

    return 0;
}
