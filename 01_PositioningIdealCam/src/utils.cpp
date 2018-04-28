#include "utils.h"

using namespace utils;

using std::vector;
using std::string;
using std::swap;
using std::sort;
using std::ifstream;
using std::ofstream;
using std::ios;
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

int utils::checkInput(int argc)
{
    if (argc == 1)
    {
        cerr << "Incorrect input format" << endl;
        return 1;
    }

    return 0;
}

int utils::unknownComand()
{
    cerr << "Unknown command" << endl;
    return 1;
}

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

double utils::randomGaussian(double radius)
{
    static bool ready = false;
    static double val;

    double ans;

    if (ready)
    {
        ans = val;
        ready = false;
    } else {
        double u, v, s;
        do {
            u = 2.0 * randomDouble() - 1.0;
            v = 2.0 * randomDouble() - 1.0;
            s = u * u + v * v;
        } while (s > 1 || s < 1e-6);

        double r = sqrt(-2.0 * log(s) / s);
        ans = r * u;
        val = r * v;
        ready = true;
    }

    return ans * radius;
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
    static vector<size_t> perm;

    if (!inited || (of != 0 && perm.size() != of))
    {
        perm.resize(of);
        iota(perm.begin(), perm.end(), 0);
        inited = true;
    }
    of = perm.size();

    vector<size_t> res;

    for (size_t i = 0; i < n; i++) swap(perm[i], perm[i + randomLess(of-i)]);
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

void utils::pointsPix2SRC(const CamModel &cam, const vector<cv::Point2f> &pointsPix, vectorV2 &pointsCam)
{
    pointsCam.clear();
    for (const cv::Point2f &pix : pointsPix)
        pointsCam.push_back(cam.getSRCFromImgCoordinates(pix));
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

string utils::readStr(const string &fileJsonName)
{
    ifstream in(fileJsonName);
    if (!in.is_open())
    {
        cerr << "Could not open file '" + fileJsonName + "'" << endl;
        exit(1);
    }

    string s = "", t;
    while (in >> t) s += t;

    in.close();

    return s;
}

void utils::readPointsJson(const string &filePointsName, vectorV3 &pointsWorld, vectorV2 &pointsCam)
{
    json j = json::parse(readStr(filePointsName));
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

ifstream utils::openToRead(const string &fileName, size_t &count)
{
    ifstream is(fileName, ios::in | ios::binary);
    if (!is.is_open())
    {
        cerr << "Could not open file '" + fileName + "'" << endl;
        exit(1);
    }

    is.read((char*)&count, sizeof(count));

    return is;
}

void utils::readSample(ifstream &is, Sample &s)
{
    is.read((char*)&s.err, sizeof(s.err));

    size_t n;
    is.read((char*)&n, sizeof(n));

    s.shifts.clear();
    while (n--)
    {
        double sft;
        is.read((char*)&sft, sizeof(sft));
        s.shifts.push_back(sft);
    }

    is.read((char*)&s.camX, sizeof(s.camX));
}

void utils::readGroup(std::ifstream &is, vector<Sample> &samples)
{
    size_t n;
    is.read((char*)&n, sizeof(n));

    samples.clear();
    while (n--)
    {
        Sample s;
        readSample(is, s);
        samples.push_back(s);
    }
}

void utils::writeStr(const string &fileName, const string &s)
{
    ofstream out(fileName);
    if (!out.is_open())
    {
        cerr << "Could not open file '" + fileName + "'" << endl;
        exit(1);
    }

    out << s;

    out.close();
}

void utils::writePointsJson(const string &filePointsName, const vectorV3 &pointsWorld, const vectorV2 &pointsCam)
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

    writeStr(filePointsName, j.dump());
}

void utils::writeTransformation(const string &fileTransformName, const Affine3d &Transform)
{
    ofstream out(fileTransformName);
    if (!out.is_open())
    {
        cerr << "Could not open file '" + fileTransformName + "'" << endl;
        exit(1);
    }

    out << Transform.matrix();

    out.close();
}

ofstream utils::openToWrite(const string &fileName, size_t count)
{
    ofstream os(fileName, ios::out | ios::binary);
    if (!os.is_open())
    {
        cerr << "Could not open file '" + fileName + "'" << endl;
        exit(1);
    }

    os.write((char*)&count, sizeof(count));

    return os;
}

void utils::writeSample(ofstream &os, const Sample &s)
{
    os.write((char*)&s.err, sizeof(s.err));

    size_t n = s.shifts.size();
    os.write((char*)&n, sizeof(n));

    for (const double &sft : s.shifts)
        os.write((char*)&sft, sizeof(sft));

    os.write((char*)&s.camX, sizeof(s.camX));
}

void utils::writeGroup(std::ofstream &os, const vector<Sample> &samples)
{
    size_t n = samples.size();
    os.write((char*)&n, sizeof(n));

    for (const Sample &s : samples)
        writeSample(os, s);
}