#pragma once

#include <vector>
#include <iostream>
#include <cmath>
#include <limits>
#include <algorithm>
#include <Eigen/Eigen>
#include "PointPair23d.h"
#include <libconfig.h++>

class PositionCalculator
{
private:
    static size_t MIN_POINTS_COUNT;

    static size_t ITR_STEP;
    static size_t ITR_RAND;

    static double randomAngle();
    static Eigen::Affine3d randomAffine3d();
    static double getError(const PointPair23d &point, const Eigen::Affine3d &Transform);

    static Eigen::Affine3d getNextTransformation(const std::vector<PointPair23d> &pointPairs, const Eigen::Affine3d &InitTransform);
    static Eigen::Affine3d getTransformationStep(const std::vector<PointPair23d> &pointPairs, const Eigen::Affine3d InitTransform = Eigen::Affine3d::Identity(), const bool use = false);
    static Eigen::Affine3d getTransformationRand(const std::vector<PointPair23d> &pointPairs);

public:
    PositionCalculator() =delete;

    static Eigen::Vector2d project(Eigen::Vector3d v3);
    static double getError(const std::vector<PointPair23d> &pointPairs, const Eigen::Affine3d &Transform);

    static void init();

    static Eigen::Affine3d getTransformation(const std::vector<PointPair23d> &pointPairs);
};