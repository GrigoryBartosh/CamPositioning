#pragma once

#include <cstdlib>
#include <vector>
#include <cstring>
#include <iostream>
#include <algorithm>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include "CamModel.h"
#include "PointPair23d.h"
#include <json.hpp>

namespace utils
{
    typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > vectorV2;
    typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > vectorV3;

    size_t randomLess(size_t n);
    double randomDouble();
    double randomFromTo(double from, double to);
    double randomAngle();
    Eigen::Affine3d randomAffine3d();

    Eigen::Vector2d project(Eigen::Vector3d v3);

    std::vector<size_t> getRandomSubset(size_t n, size_t of = 0);

    template <class T>
    std::vector<T> getSubset(const std::vector<T> &arr, const std::vector<size_t> &nums)
    {
        std::vector<T> res;

        for (const int &i : nums) 
            res.push_back(arr[i]);

        return res;
    }

    void pointsSRC2Pix(const CamModel &cam, const vectorV2 &pointsCam, std::vector<cv::Point2f> &pointsPix);
    void pointsPix2Calibrated(const CamModel &cam, const std::vector<cv::Point2f> &pointsPix, vectorV2 &pointsCam);
    std::vector<PointPair23d> constructPointPairs(const vectorV3 &pointsWorld, const vectorV2 &pointsCam);
    cv::Point2f point2Img(const CamModel &cam, const Eigen::Affine3d &T, Eigen::Vector3d v3);

    std::string readJson(const std::string &fileJsonName);
    void readPointsJson(const std::string &filePointsName, vectorV3 &pointsWorld, vectorV2 &pointsCam);
    void readPointsTxt(const std::string &filePointsName, vectorV3 &pointsWorld, vectorV2 &pointsCam);
    std::string makeJson(const vectorV3 &pointsWorld, const vectorV2 &pointsCam);
    void writePointsJson(const std::string &filePointsName, const vectorV3 &pointsWorld, const vectorV2 &pointsCam);
    void writeTransformation(const std::string &fileTransformName, const Eigen::Affine3d &Transform);
}