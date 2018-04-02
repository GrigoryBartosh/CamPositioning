#pragma once

#include <Eigen/Eigen>

struct PointPair23d
{
public:
    Eigen::Vector3d world;
    Eigen::Vector2d cam;

    PointPair23d(Eigen::Vector3d world, Eigen::Vector2d cam)
    :world(world), cam(cam)
    { }
};