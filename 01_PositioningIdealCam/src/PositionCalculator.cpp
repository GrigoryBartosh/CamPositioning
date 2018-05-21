#include "PositionCalculator.h"

using std::vector;
using std::cout;
using std::endl;

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Matrix3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix;
using Eigen::Affine3d;

using utils::PointPair23d;
using utils::randomAffine3d;
using utils::project;
using utils::getRandomSubset;
using utils::getSubset;

size_t PositionCalculator::MIN_POINTS_COUNT;

size_t PositionCalculator::ITR_CHECK;
double PositionCalculator::BOARD_CONVERGES;
size_t PositionCalculator::ITR_RANSAC;
size_t PositionCalculator::ITR_RANSAC_STEP_MAX;
double PositionCalculator::RANSAC_INLINER_BOARD;

bool PositionCalculator::isConverges(double sft)
{
    return sft < BOARD_CONVERGES;
}

double PositionCalculator::getError(const PointPair23d &point, const Affine3d &Transform) 
{
    Vector2d v2F = point.cam;
    Vector3d v3 = point.world;
    v3 = Transform * v3;
    Vector2d v2S = project(v3);
    return  (v2F - v2S).squaredNorm();
}

vector<size_t> PositionCalculator::getInlinersNum(const vector<PointPair23d> &pointPairs, const Affine3d &Transform)
{
    vector<size_t> nums;

    size_t n = pointPairs.size();
    for (size_t i = 0; i < n; i++)
        if (getError(pointPairs[i], Transform) < RANSAC_INLINER_BOARD)
            nums.push_back(i);

    return nums;
}

double PositionCalculator::getError(const vector<PointPair23d> &pointPairs, const Affine3d &Transform) 
{
    double err = 0;

    for (const PointPair23d &point : pointPairs)
        err += getError(point, Transform);

    return err;
}

double PositionCalculator::getAffin3dDeviation(const Eigen::Affine3d &Transform)
{
    VectorXd I(12), T(12);

    Affine3d Identity = Affine3d::Identity();
    I.segment<3>(0) = Identity.matrix().block<3,1>(0,0);
    I.segment<3>(3) = Identity.matrix().block<3,1>(0,1);
    I.segment<3>(6) = Identity.matrix().block<3,1>(0,2);
    I.segment<3>(9) = Identity.matrix().block<3,1>(0,3);

    T.segment<3>(0) = Transform.matrix().block<3,1>(0,0);
    T.segment<3>(3) = Transform.matrix().block<3,1>(0,1);
    T.segment<3>(6) = Transform.matrix().block<3,1>(0,2);
    T.segment<3>(9) = Transform.matrix().block<3,1>(0,3);

    return (T - I).squaredNorm();
}

double PositionCalculator::getNextTransformation(const vector<PointPair23d> &pointPairs, Affine3d &transform)
{
    MatrixXd A = MatrixXd::Zero(6,6);
    VectorXd b = VectorXd::Zero(6);

    for (const PointPair23d &point : pointPairs)
    {
        MatrixXd G(3,6);
        Matrix3d P;
        Vector3d X;
        double u, v;

        X = transform * point.world;
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

    double shift = getAffin3dDeviation(T);
    shift = log(shift);

    transform = T * transform;

    return shift;
}

Affine3d PositionCalculator::getTransformationStep(const vector<PointPair23d> &pointPairs, const Eigen::Affine3d *InitTransform, std::vector<double> *shifts, double *lastShift)
{
    Affine3d Transform = InitTransform == NULL ? randomAffine3d() : *InitTransform;

    for (size_t j = 0; j < ITR_CHECK; j++)
    {
        double sft = getNextTransformation(pointPairs, Transform);
        
        if (shifts != NULL) shifts->push_back(sft);
        if (lastShift != NULL) *lastShift = sft;

        if (isConverges(sft)) break;
    }

    return Transform;
}

Affine3d PositionCalculator::getTransformationRand(const vector<PointPair23d> &pointPairs) 
{
    Affine3d Transform;
    double lastShift;

    do
    {
        Transform = getTransformationStep(pointPairs, NULL, NULL, &lastShift);
    } while (isConverges(lastShift));

    return Transform;
}

Affine3d PositionCalculator::getTransformationRANSAC(const vector<PointPair23d> &pointPairs) 
{
    size_t n = pointPairs.size();
    if (n < MIN_POINTS_COUNT)
        return Affine3d::Identity();

    Affine3d resTransform;
    size_t maxCnt = 0;
    double minError = std::numeric_limits<double>::infinity();

    for (size_t i = 0; i < ITR_RANSAC; i++)
    {
        vector<size_t> inliners_num;
        vector<size_t> inliners_num_swp;
        vector<PointPair23d> inliners;
        Affine3d Transform;
        size_t itr = 0;

        inliners_num = getRandomSubset(MIN_POINTS_COUNT, n);
        inliners = getSubset(pointPairs, inliners_num);
        Transform = getTransformationRand(inliners);
        while (inliners_num != inliners_num_swp && itr < ITR_RANSAC_STEP_MAX)
        {
            if (inliners_num.size() < MIN_POINTS_COUNT) break;

            inliners = getSubset(pointPairs, inliners_num);

            Transform = getTransformationStep(inliners, &Transform, NULL, NULL);

            inliners_num_swp = getInlinersNum(pointPairs, Transform);

            swap(inliners_num, inliners_num_swp);
            itr++;
        }

        size_t cnt = inliners_num.size();
        if (cnt < maxCnt) continue;

        inliners = getSubset(pointPairs, inliners_num);
        double err = getError(inliners, Transform);
        if (maxCnt < cnt || minError > err)
        {
            maxCnt = cnt;
            minError = err;
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

    ITR_CHECK = (int)cfg.lookup("ITR_CHECK");
    BOARD_CONVERGES = cfg.lookup("BOARD_CONVERGES");
    ITR_RANSAC = (int)cfg.lookup("ITR_RANSAC");
    ITR_RANSAC_STEP_MAX = (int)cfg.lookup("ITR_RANSAC_STEP_MAX");

    //TODO
    //now it reads pixel dist
    RANSAC_INLINER_BOARD = cfg.lookup("RANSAC_INLINER_BOARD");
    RANSAC_INLINER_BOARD *= 0.0027;
    RANSAC_INLINER_BOARD *= RANSAC_INLINER_BOARD;
}