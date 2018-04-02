#include "PositionCalculator.h"

using std::vector;

using Eigen::AngleAxisd;
using Eigen::Quaternion;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Matrix3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix;
using Eigen::Affine3d;

size_t PositionCalculator::MIN_POINTS_COUNT;

size_t PositionCalculator::ITR_STEP;
size_t PositionCalculator::ITR_RAND;

double PositionCalculator::randomAngle()
{
    return 2 * M_PI * (double) rand() / RAND_MAX;
}

Affine3d PositionCalculator::randomAffine3d()
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

Vector2d PositionCalculator::project(Vector3d v3)
{
    Vector2d v2;
    v2 << v3(0) / v3(2), v3(1) / v3(2);
    return v2;
}

double PositionCalculator::getError(const PointPair23d &point, const Affine3d &Transform) 
{
    Vector2d v2F = point.cam;
    Vector3d v3 = point.world;
    v3 = Transform * v3;
    Vector2d v2S = project(v3);
    double err = (v2F - v2S).norm();
    return err * err;
}

double PositionCalculator::getError(const vector<PointPair23d> &pointPairs, const Affine3d &Transform) 
{
    double err = 0;

    for (const PointPair23d &point : pointPairs)
        err += getError(point, Transform);

    return err;
}

Affine3d PositionCalculator::getNextTransformation(const vector<PointPair23d> &pointPairs, const Affine3d &InitTransform)
{
    MatrixXd A = MatrixXd::Zero(6,6);
    VectorXd b = VectorXd::Zero(6);

    for (const PointPair23d &point : pointPairs)
    {
        MatrixXd G(3,6);
        Matrix3d P;
        Vector3d X;
        double u, v;

        X = InitTransform * point.world;
        u = point.cam(0);
        v = point.cam(1);

        P <<  0, -1,  v,
              1,  0, -u,
             -v,  u,  0;

        G << 1, 0, 0,     0,  X(2), -X(1),
             0, 1, 0, -X(2),     0,  X(0),
             0, 0, 1,  X(1), -X(0),     0;

        A += G.transpose() * P.transpose() * P * G;
        b -= G.transpose() * P.transpose() * P * X;
    }

    Matrix<double, 6, 1> v = A.colPivHouseholderQr().solve(b);

    Vector3d w;
    w << v(3), v(4), v(5);
    double theta = w.norm();
    Matrix3d Wx;
    Wx <<     0, -w(2),  w(1),
           w(2),     0, -w(0),
          -w(1),  w(0),     0;

    Matrix3d new_rotation = Matrix3d::Identity() + sin(theta) / theta * Wx + (1 - cos(theta)) / (theta * theta) * Wx * Wx;

    Affine3d T;
    T.linear() = new_rotation;
    for (size_t i = 0; i < 3; i++) T.translation()(i) = v(i);

    return T * InitTransform;
}

Affine3d PositionCalculator::getTransformationStep(const vector<PointPair23d> &pointPairs, const Eigen::Affine3d InitTransform, const bool use) 
{
    Affine3d Transform = use ? InitTransform : randomAffine3d();

    for (size_t j = 0; j < ITR_STEP; j++)
        Transform = getNextTransformation(pointPairs, Transform);   

    return Transform;
}

Affine3d PositionCalculator::getTransformationRand(const vector<PointPair23d> &pointPairs) 
{
    Affine3d resTransform;
    double minError = std::numeric_limits<double>::infinity();

    for (size_t i = 0; i < ITR_RAND; i++)
    {
        Affine3d Transform = getTransformationStep(pointPairs);

        double error = getError(pointPairs, Transform);

        if (minError > error)
        {
            minError = error;
            resTransform = Transform;
        }
    }

    return resTransform;
}

void PositionCalculator::init()
{
    libconfig::Config cfg;
    cfg.readFile("config/PositionCalculator.cfg");

    MIN_POINTS_COUNT = (int)cfg.lookup("MIN_POINTS_COUNT");

    ITR_STEP = (int)cfg.lookup("ITR_STEP");
    ITR_RAND = (int)cfg.lookup("ITR_RAND");
}

Affine3d PositionCalculator::getTransformation(const vector<PointPair23d> &pointPairs) 
{
    if (pointPairs.size() < MIN_POINTS_COUNT)
        return Affine3d::Identity();

    return getTransformationRand(pointPairs);
}