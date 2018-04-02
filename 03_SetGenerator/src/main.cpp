#include <iostream>
#include <cstdlib>
#include "CamModel.h"
#include <json.hpp>

using namespace std;
using namespace Eigen;

using json = nlohmann::json;

const string FILE_JSON_NAME = "points.json";
const string FILE_TRANSFORM_NAME = "transformation.txt";

const size_t POINTS_COUNT = 20;
const double MAX_LEN = 100;

const size_t CAM_W = 1280;
const size_t CAM_H = 720;
const double CAM_ANGLE_X = 60 * M_PI / 180;

typedef vector<Vector2d, aligned_allocator<Vector2d> > vectorV2;
typedef vector<Vector3d, aligned_allocator<Vector3d> > vectorV3;

string makeJson(const vectorV3 &pointsWorld, const vectorV2 &pointsCam)
{
    json j;
    size_t n = pointsWorld.size();
    for (size_t i = 0; i < n; i++)
    {
        const Vector2d &v2 = pointsCam[i];
        j[i]["point2D"]["x"] = v2.x();
        j[i]["point2D"]["y"] = v2.y();

        const Vector3d &v3 = pointsWorld[i];
        j[i]["point3D"]["x"] = v3.x();
        j[i]["point3D"]["y"] = v3.y();
        j[i]["point3D"]["z"] = v3.z();
    }

    return j.dump();
}

void writePoints(const vectorV3 &pointsWorld, const vectorV2 &pointsCam)
{
    string j = makeJson(pointsWorld, pointsCam);

    ofstream out(FILE_JSON_NAME);
    if (!out.is_open())
    {
        cerr << "Could not open file '" + FILE_JSON_NAME + "'";
        exit(1);
    }

    out << j;

    out.close();
}

void writeTransform(const Affine3d &Transform)
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

double rnd()
{
    return (double) rand() / RAND_MAX;
}

double rnd(double l, double r)
{
    return l + (r - l) * rnd();
}

double rndAngle()
{
    return 2 * M_PI * (double) rand() / RAND_MAX;
}

void generateSRC(vectorV2 &pointsSrc)
{
    for (size_t i = 0; i < POINTS_COUNT; i++)
    {
        Vector2d v2;
        v2 << rnd(-1, 1), rnd(-1, 1);
        pointsSrc.push_back(v2);
    }
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

void generateTransformation(Affine3d &Transform)
{
    AngleAxisd rollAngle(rndAngle(), Vector3d::UnitZ());
    AngleAxisd yawAngle(rndAngle(), Vector3d::UnitY());
    AngleAxisd pitchAngle(rndAngle(), Vector3d::UnitX());

    Quaternion<double> q = rollAngle * yawAngle * pitchAngle;

    Matrix3d rotationMatrix = q.matrix();

    Vector3d shift = MatrixXd::Random(3,1);

    Transform = Affine3d::Identity();
    Transform.matrix().block<3,3>(0,0) = rotationMatrix;
    Transform.matrix().block<3,1>(0,3) = shift;
}

void generateWorld(const Affine3d &Transform, const vectorV2 &pointsCam, vectorV3 &pointsWorld)
{
    Affine3d TInv = Transform.inverse();
    for (const Vector2d &v2 : pointsCam)
    {
        Vector3d v3;
        v3 << v2.x(), v2.y(), 1;
        v3 *= rnd() * MAX_LEN;
        v3 = TInv * v3;
        pointsWorld.push_back(v3);
    }
}

void draw(const Affine3d &Transform, const vector<cv::Point2f> &pointsPix)
{
    cout << Transform.matrix() << endl;

    cv::Mat frame(CAM_H, CAM_W, CV_8UC3);

    for (const cv::Point2f &pix : pointsPix)
        cv::circle(frame, cv::Point(pix.x, pix.y), 3, cv::Scalar(0, 255, 0), -1);

    imwrite("img.jpg", frame);
    cv::imshow("img", frame);
    cv::waitKey();
}

int main()
{
    srand(time(NULL));

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

    generateTransformation(Transform);
    generateWorld(Transform, pointsCam, pointsWorld);

    writePoints(pointsWorld, pointsSrc);    
    writeTransform(Transform);

    draw(Transform, pointsPix);

    return 0;
}
