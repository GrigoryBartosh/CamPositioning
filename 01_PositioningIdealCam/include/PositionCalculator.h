#pragma once

#include <vector>
#include <cmath>
#include <limits>
#include <Eigen/Eigen>
#include "utils.h"
#include <libconfig.h++>

class PositionCalculator
{
private:
    static size_t MIN_POINTS_COUNT;

    static size_t ITR_CHECK;
    static double BOARD_CONVERGES;
    static size_t ITR_RANSAC;
    static size_t ITR_RANSAC_STEP_MAX;
    static double RANSAC_INLINER_BOARD;

    static bool isConverges(double sft);

    static double getError(const utils::PointPair23d &point, const Eigen::Affine3d &Transform);
    static double getAffin3dDeviation(const Eigen::Affine3d &Transform);

    static double getNextTransformation(const std::vector<utils::PointPair23d> &pointPairs, Eigen::Affine3d &InitTransform);

public:
    PositionCalculator() =delete;

    static double getError(const std::vector<utils::PointPair23d> &pointPairs, const Eigen::Affine3d &Transform);
    static std::vector<size_t> getInlinersNum(const std::vector<utils::PointPair23d> &pointPairs, const Eigen::Affine3d &Transform);

    static void init();

    static Eigen::Affine3d getTransformationStep(const std::vector<utils::PointPair23d> &pointPairs, const Eigen::Affine3d *InitTransform, std::vector<double> *shifts, double *lastShift);
    static Eigen::Affine3d getTransformationRand(const std::vector<utils::PointPair23d> &pointPairs);
    static Eigen::Affine3d getTransformationRANSAC(const std::vector<utils::PointPair23d> &pointPairs);
};