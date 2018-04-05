#pragma once

#include <vector>
#include <limits>
#include <Eigen/Eigen>
#include "PointPair23d.h"
#include "utils.h"
#include <libconfig.h++>

class PositionCalculator
{
private:
    static size_t MIN_POINTS_COUNT;

    static size_t ITR_STEP;
    static size_t ITR_RAND;
    static size_t ITR_RANSAC;
    static size_t ITR_RANSAC_STEP_MAX;
    static double RANSAC_INLINER_BOARD;

    static double getError(const PointPair23d &point, const Eigen::Affine3d &Transform);

    static Eigen::Affine3d getNextTransformation(const std::vector<PointPair23d> &pointPairs, const Eigen::Affine3d &InitTransform);

public:
    PositionCalculator() =delete;

    static double getError(const std::vector<PointPair23d> &pointPairs, const Eigen::Affine3d &Transform);
    static std::vector<size_t> getInlinersNum(const std::vector<PointPair23d> &pointPairs, const Eigen::Affine3d &Transform);

    static void init();

    static Eigen::Affine3d getTransformationStep(const std::vector<PointPair23d> &pointPairs, const Eigen::Affine3d InitTransform = Eigen::Affine3d::Identity(), const bool use = false);
    static Eigen::Affine3d getTransformationRand(const std::vector<PointPair23d> &pointPairs);
    static Eigen::Affine3d getTransformationRANSAC(const std::vector<PointPair23d> &pointPairs);
};