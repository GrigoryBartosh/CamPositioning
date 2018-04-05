#include "utils.h"

using namespace utils;

using std::vector;
using std::string;
using std::swap;
using std::sort;
using std::ifstream;
using std::ofstream;
using std::cerr;
using std::endl;

using Eigen::AngleAxisd;
using Eigen::Quaternion;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Affine3d;

using json = nlohmann::json;

size_t utils::randomLess(size_t n)
{
    return rand() % n;
}

double utils::randomDouble()
{
    return (double) rand() / RAND_MAX;
}

double utils::randomFromTo(double from, double to)
{
    return from + (to - from) * randomDouble();
}

double utils::randomAngle()
{
    return 2 * M_PI * (double) rand() / RAND_MAX;
}

Affine3d utils::randomAffine3d()
{
    AngleAxisd rollAngle(randomAngle(), Vector3d::UnitZ());
    AngleAxisd yawAngle(randomAngle(), Vector3d::UnitY());
    AngleAxisd pitchAngle(randomAngle(), Vector3d::UnitX());

    Quaternion<double> q = rollAngle * yawAngle * pitchAngle;

    Matrix3d rotationMatrix = q.matrix();

    Vector3d shift = MatrixXd::Random(3,1);

    Affine3d Transform = Affine3d::Identity();
    Transform.matrix().block<3,3>(0,0) = rotationMatrix;
    Transform.matrix().block<3,1>(0,3) = shift;

    return Transform;
}

Vector2d utils::project(Vector3d v3)
{
    Vector2d v2;
    v2 << v3.x() / v3.z(), v3.y() / v3.z();
    return v2;
}

vector<size_t> utils::getRandomSubset(size_t n, size_t of)
{
    static bool inited = false;
    static std::vector<size_t> perm;

    if (!inited || (of != 0 && perm.size() != of))
    {
        perm.resize(n);
        std::iota(perm.begin(), perm.end(), 0);
        inited = true;
    }

    vector<size_t> res;

    for (size_t i = 0; i < n; i++) swap(perm[i], perm[i + randomLess(n-i)]);
    for (size_t i = 0; i < n; i++) res.push_back(perm[i]);
    sort(res.begin(), res.end());

    return res;
}

void utils::pointsSRC2Pix(const CamModel &cam, const vectorV2 &pointsCam, vector<cv::Point2f> &pointsPix)
{
    pointsPix.clear();
    for (const Vector2d &v2 : pointsCam)
        pointsPix.push_back(cam.getImgCoordinatesFromSRC(v2));
}

void utils::pointsPix2Calibrated(const CamModel &cam, const vector<cv::Point2f> &pointsPix, vectorV2 &pointsCam)
{
    pointsCam.clear();
    for (const cv::Point2f &pix : pointsPix)
        pointsCam.push_back(cam.getCalibratedCoordinates(pix));
}

vector<PointPair23d> utils::constructPointPairs(const vectorV3 &pointsWorld, const vectorV2 &pointsCam)
{
    vector<PointPair23d> pointPairs;

    size_t n = pointsWorld.size();
    for (size_t i = 0; i < n; i++)
        pointPairs.push_back(PointPair23d(pointsWorld[i], pointsCam[i]));

    return pointPairs;
}

cv::Point2f utils::point2Img(const CamModel &cam, const Affine3d &T, Vector3d v3)
{
    v3 = T * v3;
    Vector2d v2 = utils::project(v3);
    cv::Point2f p = cam.getImgCoordinates(v2);
    return p;
}

string utils::readJson(const string &fileJsonName)
{
    ifstream in(fileJsonName);
    if (!in.is_open())
    {
        cerr << "Could not open file '" + fileJsonName + "'";
        exit(1);
    }

    string s = "", t;
    while (in >> t) s += t;

    in.close();

    return s;
}

void utils::readPointsJson(const string &filePointsName, vectorV3 &pointsWorld, vectorV2 &pointsCam)
{
    string s = readJson(filePointsName);
    json j = json::parse(s);
    for (size_t i = 0; i < j.size(); i++)
    {
        Vector2d v2;
        v2 << j[i]["point2D"]["x"], j[i]["point2D"]["y"];

        Vector3d v3;
        v3 << j[i]["point3D"]["x"], j[i]["point3D"]["y"], j[i]["point3D"]["z"];

        pointsWorld.push_back(v3);
        pointsCam.push_back(v2);
    }
}

void utils::readPointsTxt(const string &filePointsName, vectorV3 &pointsWorld, vectorV2 &pointsCam)
{
    ifstream in(filePointsName);
    if (!in.is_open())
    {
        cerr << "Could not open file '" + filePointsName + "'";
        exit(1);
    }
    
    Vector3d v3;
    Vector2d v2;
    while (in >> v3.x())
    {
        in >> v3.y() >> v3.z();
        in >> v2.x() >> v2.y();

        pointsWorld.push_back(v3);
        pointsCam.push_back(v2);
    }

    in.close();
}

string utils::makeJson(const vectorV3 &pointsWorld, const vectorV2 &pointsCam)
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

void utils::writePointsJson(const string &filePointsName, const vectorV3 &pointsWorld, const vectorV2 &pointsCam)
{
    string j = makeJson(pointsWorld, pointsCam);

    ofstream out(filePointsName);
    if (!out.is_open())
    {
        cerr << "Could not open file '" + filePointsName + "'";
        exit(1);
    }

    out << j;

    out.close();
}

void utils::writeTransformation(const string &fileTransformName, const Affine3d &Transform)
{
    ofstream out(fileTransformName);
    if (!out.is_open())
    {
        cerr << "Could not open file '" + fileTransformName + "'";
        exit(1);
    }

    out << Transform.matrix();

    out.close();
}