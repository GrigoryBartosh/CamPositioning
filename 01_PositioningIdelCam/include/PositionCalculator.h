#pragma once

#include <vector>
#include <iostream>
#include <cmath>
#include <limits>
#include <Eigen/Eigen>

class PositionCalculator
{
private:
    static const size_t ITR_STEP = 20;
    static const size_t ITR_RAND = 100;
    static const size_t MIN_POINTS_COUNT = 6;

    typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> vectorV2;
    typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> vectorV3;

    static double randomAngle();
    static Eigen::Affine3d randomAffine3d();
    static Eigen::Affine3d getNextTransform(const vectorV3 &pointsWorld, const vectorV2 &pointsCam, const Eigen::Affine3d &InitTransform);

public:
    PositionCalculator() =delete;

    static Eigen::Vector2d project(Eigen::Vector3d v3);

    static Eigen::Affine3d getTransformation(const vectorV3 &pointsWorld, const vectorV2 &pointsCam);
    static double getError(const vectorV3 &pointsWorld, const vectorV2 &pointsCam, const Eigen::Affine3d &Transform);
};