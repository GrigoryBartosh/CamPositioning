#pragma once

#include <cstdlib>
#include <vector>
#include <cstring>
#include <iostream>
#include <algorithm>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include "CamModel.h"
#include <json.hpp>

namespace utils
{
    typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> vectorV2;
    typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> vectorV3;

    struct PointPair23d
    {
        Eigen::Vector3d world;
        Eigen::Vector2d cam;

        PointPair23d(Eigen::Vector3d world, Eigen::Vector2d cam)
        :world(world), cam(cam)
        { }
    };

    struct Sample
    {
        double err;
        std::vector<double> shifts;
        double camX;

        Sample() =default;

        Sample(const Sample &other)
        {
            err = other.err;
            shifts = other.shifts;
            camX = other.camX;
        }

        Sample(Sample &&other)
        {
            std::swap(err, other.err);
            std::swap(shifts, other.shifts);
            std::swap(camX, other.camX);
        }
    };

    int checkInput(int argc);
    int unknownComand();

    size_t randomLess(size_t n);
    double randomDouble();
    double randomFromTo(double from, double to);
    double randomGaussian(double radius);
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
    void pointsPix2SRC(const CamModel &cam, const std::vector<cv::Point2f> &pointsPix, vectorV2 &pointsCam);
    void pointsPix2Calibrated(const CamModel &cam, const std::vector<cv::Point2f> &pointsPix, vectorV2 &pointsCam);
    std::vector<PointPair23d> constructPointPairs(const vectorV3 &pointsWorld, const vectorV2 &pointsCam);
    cv::Point2f point2Img(const CamModel &cam, const Eigen::Affine3d &T, Eigen::Vector3d v3);

    std::string readStr(const std::string &fileJsonName);
    void readPointsJson(const std::string &filePointsName, vectorV3 &pointsWorld, vectorV2 &pointsCam);
    std::ifstream openToRead(const std::string &fileName, size_t &count);
    void readSample(std::ifstream &is, Sample &s);
    void readGroup(std::ifstream &is, std::vector<Sample> &samples);
    void writeStr(const std::string &fileName, const std::string &s);
    void writePointsJson(const std::string &filePointsName, const vectorV3 &pointsWorld, const vectorV2 &pointsCam);
    void writeTransformation(const std::string &fileTransformName, const Eigen::Affine3d &Transform);
    std::ofstream openToWrite(const std::string &fileName, size_t count);
    void writeSample(std::ofstream &os, const Sample &s);
    void writeGroup(std::ofstream &os, const std::vector<Sample> &samples);
}