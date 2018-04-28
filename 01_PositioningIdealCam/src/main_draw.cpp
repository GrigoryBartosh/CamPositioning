#include "my_algorythms.h"

#include <iostream>
#include <cmath>
#include <opencv2/opencv.hpp>
#include "utils.h"
#include <libconfig.h++>

using namespace std;
using namespace Eigen;

using utils::Sample;
using utils::PointPair23d;

using utils::openToRead;
using utils::readSample;
using utils::readGroup;

static string FILE_SAMPLES_SINGLE_NAME;
static string FILE_SAMPLES_GROUP_NAME;
static string FILE_IMG_CONVERGENCE_NAME;
static string FILE_IMG_LINES_NAME;
static string FILE_IMG_DISTRIBUTION_NAME;
static string FILE_IMG_DISTRIBUTIONS_ALL_NAME;
static string FILE_IMG_POSITIONING_NAME;
static string FILE_IMG_PROBABILITY_GOOD_NAME;

static size_t ITERATIONS;
static size_t ITERATION_CHECK;

static double BOARD_GOOD;
static double BOARD_CONVERGES;

static size_t ITERATION_STEP;
static size_t CONVERGENCE_H;
static size_t DISTRIBUTION_H;
static size_t DISTRIBUTION_RESOLUTION;
static int DISTRIBUTION_FROM;
static int DISTRIBUTION_TO;

static size_t PROBABILITY_GOOD_RESOLUTION;
static size_t PROBABILITY_GOOD_W;
static size_t PROBABILITY_GOOD_H;

static size_t POSITIONING_W;
static size_t POSITIONING_H;
static double POSITIONING_TO;

static size_t CAM_W;
static size_t CAM_H;
static double CAM_ANGLE_X;

static void readConfig()
{
    libconfig::Config cfg;
    cfg.readFile("config/main_draw.cfg");

    FILE_SAMPLES_SINGLE_NAME = cfg.lookup("FILE_SAMPLES_SINGLE_NAME").c_str();
    FILE_SAMPLES_GROUP_NAME = cfg.lookup("FILE_SAMPLES_GROUP_NAME").c_str();
    FILE_IMG_CONVERGENCE_NAME = cfg.lookup("FILE_IMG_CONVERGENCE_NAME").c_str();
    FILE_IMG_LINES_NAME = cfg.lookup("FILE_IMG_LINES_NAME").c_str();
    FILE_IMG_DISTRIBUTION_NAME = cfg.lookup("FILE_IMG_DISTRIBUTION_NAME").c_str();
    FILE_IMG_DISTRIBUTIONS_ALL_NAME = cfg.lookup("FILE_IMG_DISTRIBUTIONS_ALL_NAME").c_str();
    FILE_IMG_POSITIONING_NAME = cfg.lookup("FILE_IMG_POSITIONING_NAME").c_str();
    FILE_IMG_PROBABILITY_GOOD_NAME = cfg.lookup("FILE_IMG_PROBABILITY_GOOD_NAME").c_str();

    ITERATIONS = (int)cfg.lookup("ITERATIONS");
    ITERATION_CHECK = (int)cfg.lookup("ITERATION_CHECK");

    BOARD_GOOD = cfg.lookup("BOARD_GOOD");
    BOARD_CONVERGES = cfg.lookup("BOARD_CONVERGES");

    ITERATION_STEP = (int)cfg.lookup("ITERATION_STEP");
    CONVERGENCE_H = (int)cfg.lookup("CONVERGENCE_H");
    DISTRIBUTION_H = (int)cfg.lookup("DISTRIBUTION_H");
    DISTRIBUTION_RESOLUTION = (int)cfg.lookup("DISTRIBUTION_RESOLUTION");
    DISTRIBUTION_FROM = cfg.lookup("DISTRIBUTION_FROM");
    DISTRIBUTION_TO = cfg.lookup("DISTRIBUTION_TO");

    PROBABILITY_GOOD_RESOLUTION = (int)cfg.lookup("PROBABILITY_GOOD_RESOLUTION");
    PROBABILITY_GOOD_W = (int)cfg.lookup("PROBABILITY_GOOD_W");
    PROBABILITY_GOOD_H = (int)cfg.lookup("PROBABILITY_GOOD_H");

    POSITIONING_W = (int)cfg.lookup("POSITIONING_W");
    POSITIONING_H = (int)cfg.lookup("POSITIONING_H");
    POSITIONING_TO = cfg.lookup("POSITIONING_TO");

    cfg.readFile("config/camera.cfg");
    CAM_W = (int)cfg.lookup("CAM_W");
    CAM_H = (int)cfg.lookup("CAM_H");
    CAM_ANGLE_X = cfg.lookup("CAM_ANGLE_X");
    CAM_ANGLE_X *= M_PI / 180;
}

static void init()
{
    srand(time(NULL));

    readConfig();
}

static bool isGood(const Sample &s)
{
    return s.err < BOARD_GOOD;
}

static bool isCoverge(double sft)
{
    return sft < BOARD_CONVERGES;
}

static vector<double> getNorm(const vector<size_t> v)
{
    size_t maxCnt = 0;
    for (const size_t &cnt : v)
        maxCnt = max(maxCnt, cnt);

    vector<double> vNorm;
    for (const size_t &cnt : v)
        vNorm.push_back((double) cnt / maxCnt);

    return vNorm;
}

static void drawGraph(cv::Mat &img, const vector<double> &ys, double step, cv::Scalar color)
{
    size_t h = img.size().height;

    for (size_t i = 1; i < ys.size(); i++)
    {
        long double y1 = ys[i-1];
        long double y2 = ys[i];

        cv::line(img, cv::Point((i-1) * step, (1-y1) * h), 
                      cv::Point( i    * step, (1-y2) * h), color);
    }
}

static void drawContinuousLine(cv::Mat &img, size_t pos, bool horizontal)
{
    size_t w = img.size().width;
    size_t h = img.size().height;

    cv::Point p1, p2;

    p1.x = horizontal ? 0 : pos;
    p1.y = horizontal ? h - 1 - pos : 0;

    p2.x = horizontal ? w : pos;
    p2.y = horizontal ? h - 1 - pos : h;

    cv::line(img, p1, p2, cv::Scalar(0, 255, 0));
}

static double normCoord(double x, double from, double to)
{
    return (x - from) / (to - from);
}

static double coordToDistribution(double x)
{
    return normCoord(x, DISTRIBUTION_FROM, DISTRIBUTION_TO);
}

static string addFileIndex(string s, size_t i)
{
    size_t pos = s.find_last_of(".");
    return s.substr(0, pos) + "_" + to_string(i) + s.substr(pos);
}

static void updateConvergence(vector<size_t> &convergence, const Sample &sample)
{
    if (!isGood(sample)) return;

    size_t itr = 0;
    while (itr < sample.shifts.size() && itr < ITERATIONS && sample.shifts[itr] > BOARD_CONVERGES) itr++;

    convergence[itr]++;
}

static void drawConvergence(const vector<size_t> &convergence, double prGood)
{
    vector<double> convergenceNorm = getNorm(convergence);

    cv::Mat img(CONVERGENCE_H, ITERATION_STEP * (ITERATIONS-1), CV_8UC3, cv::Scalar(0,0,0));

    drawGraph(img, convergenceNorm, ITERATION_STEP, cv::Scalar(255, 0, 0));

    drawContinuousLine(img, (ITERATION_CHECK-0.5) * ITERATION_STEP, false);

    size_t eItr = 0;
    size_t prOutTime = 0;
    size_t sum = 0;
    for (size_t i = 0; i < ITERATION_CHECK; i++)
    {
        eItr += (i+1) * convergence[i];
        sum += convergence[i];
    }
    for (size_t i = ITERATION_CHECK; i < ITERATIONS; i++)
    {
        eItr += ITERATION_CHECK * convergence[i];
        prOutTime += convergence[i];
        sum += convergence[i];
    }

    double EItrNorm = (double) eItr / sum;
    double prOutTimeNorm = (double) prOutTime / sum;
    double ETotalItr = ((1-prGood) * ITERATION_CHECK + prGood * EItrNorm) / (prGood - prOutTimeNorm);

    cout << "E[itr total] = " << ETotalItr << endl;

    cv::putText(img, "E[itr for good] = " + to_string(EItrNorm), 
                cv::Point(img.size().width - 300, 20), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
    cv::putText(img, "Pr[out of time] = " + to_string(prOutTimeNorm), 
                cv::Point(img.size().width - 300, 40), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
    cv::putText(img, "E[itr total] = " + to_string(ETotalItr), 
                cv::Point(img.size().width - 300, 60), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));

    cv::imshow("convergence", img);
    imwrite(FILE_IMG_CONVERGENCE_NAME, img);
}

static void drawBoardLines(cv::Mat &img)
{
    double boardConvergesPos = coordToDistribution(BOARD_CONVERGES) * DISTRIBUTION_RESOLUTION;
    drawContinuousLine(img, boardConvergesPos, img.size().height == (int)DISTRIBUTION_RESOLUTION);
}

static void updateLines(cv::Mat &img, const Sample &sample)
{
    cv::Scalar color;
    if (isGood(sample)) color = cv::Scalar(255, 0, 0);
    else                color = cv::Scalar(0, 0, 255);

    vector<double> shiftsNorm;
    for (const double &sft : sample.shifts)
        shiftsNorm.push_back(coordToDistribution(sft));

    drawGraph(img, shiftsNorm, ITERATION_STEP, color);
}

static void drawLines(cv::Mat &img)
{
    drawBoardLines(img);

    cv::imshow("lines", img);
    imwrite(FILE_IMG_LINES_NAME, img);
}

static void updateDistribution(vector<size_t> &distribution, double sft)
{
    double posNorm = coordToDistribution(sft) * DISTRIBUTION_RESOLUTION;
    int pos = round(posNorm);

    if (pos < 0 || (int)DISTRIBUTION_RESOLUTION <= pos) return;

    distribution[pos]++;
}

static void updateDistributions(vector<vector<vector<size_t>>> &distributions, vector<vector<size_t>> &prRecognize, vector<size_t> &recognizeSum, const Sample &sample)
{
    size_t type = 1 - isGood(sample);

    for (size_t i = 0; i < ITERATIONS; i++)
    {
        double sft = sample.shifts.size() > i ? sample.shifts[i] : sample.shifts.back();
        
        updateDistribution(distributions[i][type], sft);

        prRecognize[i][type] += isCoverge(sft);
    }

    recognizeSum[type]++;
}

static vector<vector<double>> getNormPrRecognize(const vector<vector<size_t>> &prRecognize, const vector<size_t> &recognizeSum)
{
    vector<vector<double>> prRecognizeNorm(ITERATIONS, vector<double>(2));
    for (size_t i = 0; i < ITERATIONS; i++)
        for (size_t j = 0; j < 2; j++)
            prRecognizeNorm[i][j] = (double) prRecognize[i][j] / recognizeSum[j];

    return prRecognizeNorm;
}

static void drawLineAddColor(cv::Mat &img, size_t x1, size_t x2, size_t y, cv::Scalar color)
{
    for (size_t x = x1; x < x2; x++)
    {
        cv::Vec3b c = img.at<cv::Vec3b>(cv::Point(x,y));

        for (size_t i = 0; i < 3; i++)
            c.val[i] += color.val[i];

        img.at<cv::Vec3b>(cv::Point(x,y)) = c;
    }
}

static void drawDistributionForITR(cv::Mat &imgDistribution, cv::Mat &imgDistributionsAll, const vector<double> &distribution, size_t numItr, cv::Scalar color)
{
    drawGraph(imgDistribution, distribution, 1, color);

    for (size_t py = 0; py < DISTRIBUTION_RESOLUTION; py++)
    {
        double cnt = distribution[py];

        cv::Scalar lColor;
        for (size_t i = 0; i < 3; i++)
            lColor.val[i] = color.val[i] * cnt;

        drawLineAddColor(imgDistributionsAll, numItr * ITERATION_STEP, (numItr + 1) * ITERATION_STEP, DISTRIBUTION_RESOLUTION - 1 - py, lColor);
    }
}

static void drawDistribution(cv::Mat &imgDistributionsAll, const vector<vector<double>> &distribution, const vector<double> &prRecognize, size_t num)
{
    cv::Mat imgDistribution(DISTRIBUTION_H, DISTRIBUTION_RESOLUTION, CV_8UC3, cv::Scalar(0,0,0));

    drawDistributionForITR(imgDistribution, imgDistributionsAll, distribution[0], num, cv::Scalar(255, 0, 0));
    drawDistributionForITR(imgDistribution, imgDistributionsAll, distribution[1], num, cv::Scalar(0, 0, 255));

    drawBoardLines(imgDistribution);

    cv::putText(imgDistribution, "Pr[recognize good] = " + to_string(prRecognize[0]), 
                cv::Point(DISTRIBUTION_RESOLUTION - 300, 20), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
    cv::putText(imgDistribution, "Pr[recognize bad] = " + to_string(prRecognize[1]), 
                cv::Point(DISTRIBUTION_RESOLUTION - 300, 40), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));

    string fileName = addFileIndex(FILE_IMG_DISTRIBUTION_NAME, num + 1);
    imwrite(fileName, imgDistribution);
}

static void drawDistributions(const vector<vector<vector<size_t>>> &distributions, const vector<vector<double>> &prRecognize)
{
    vector<vector<vector<double>>> distributionsNorm(ITERATIONS, vector<vector<double>>(2, vector<double>(DISTRIBUTION_RESOLUTION)));
    for (size_t i = 0; i < ITERATIONS; i++)
    {
        size_t maxCnt = 0;
        for (const auto &b : distributions[i])
        for (const size_t &cnt : b)
            maxCnt = max(maxCnt, cnt);

        for (size_t j = 0; j < 2; j++)
        for (size_t k = 0; k < DISTRIBUTION_RESOLUTION; k++)
            distributionsNorm[i][j][k] = (double)distributions[i][j][k] / maxCnt;    
    }

    cv::Mat imgDistributionsAll(DISTRIBUTION_RESOLUTION, ITERATION_STEP * ITERATIONS, CV_8UC3, cv::Scalar(0,0,0));

    for (size_t i = 0; i < ITERATIONS; i++)
        drawDistribution(imgDistributionsAll, distributionsNorm[i], prRecognize[i], i);
    
    drawBoardLines(imgDistributionsAll);

    cv::imshow("distributions", imgDistributionsAll);
    imwrite(FILE_IMG_DISTRIBUTIONS_ALL_NAME, imgDistributionsAll);
}

static void updatePositioning(vector<size_t> &positioning, const Sample &s)
{
    if (!isGood(s)) return;

    double x = s.camX;
    int pos = round(normCoord(x, -POSITIONING_TO, POSITIONING_TO) * POSITIONING_W);

    if (pos < 0 || (int)POSITIONING_W <= pos) return;

    positioning[pos]++;
}

static void drawPositioning(const vector<size_t> &positioning)
{
    vector<double> positioningNorm = getNorm(positioning);

    cv::Mat img(POSITIONING_H, POSITIONING_W, CV_8UC3, cv::Scalar(0,0,0));

    drawGraph(img, positioningNorm, 1, cv::Scalar(0, 0, 255));

    cv::imshow("positioning", img);
    imwrite(FILE_IMG_POSITIONING_NAME, img);
}

static double updateProbabilityGood(vector<size_t> &probabilityGood, const vector<Sample> &group)
{
    size_t cnt = 0;
    for (const Sample &sample : group)
        if (isGood(sample))
            cnt++;

    double pr = (double) cnt / group.size();
    int pos = round(pr * PROBABILITY_GOOD_RESOLUTION);

    if (pos < 0 || (int)PROBABILITY_GOOD_RESOLUTION <= pos) return 0;

    probabilityGood[pos]++;

    return pr;
}

static void drawProbabilityGood(const vector<size_t> &probabilityGood, double prGood)
{
    vector<double> probabilityGoodNorm = getNorm(probabilityGood);

    cv::Mat img(PROBABILITY_GOOD_H, PROBABILITY_GOOD_W, CV_8UC3, cv::Scalar(0,0,0));

    drawGraph(img, probabilityGoodNorm, (double) PROBABILITY_GOOD_W / PROBABILITY_GOOD_RESOLUTION, cv::Scalar(255, 0, 0));

    cv::putText(img, "Pr[get good] = " + to_string(prGood), cv::Point(PROBABILITY_GOOD_W - 250, 20), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));

    cv::imshow("probability_good", img);
    imwrite(FILE_IMG_PROBABILITY_GOOD_NAME, img);
}

int my_algorythms::main_draw()
{
    init();

    vector<size_t> convergence(ITERATIONS);
    cv::Mat imgLines(DISTRIBUTION_RESOLUTION, ITERATION_STEP * (ITERATIONS-1), CV_8UC3, cv::Scalar(0,0,0));
    vector<vector<size_t>> prRecognize(ITERATIONS, vector<size_t>(2));
    vector<size_t> recognizeSum(2);
    vector<vector<vector<size_t>>> distributions(ITERATIONS, vector<vector<size_t>>(2, vector<size_t>(DISTRIBUTION_RESOLUTION)));
    vector<size_t> positioning(POSITIONING_W);

    size_t samplesCount;
    ifstream is = openToRead(FILE_SAMPLES_SINGLE_NAME, samplesCount);
    for (size_t i = 0; i < samplesCount; i++)
    {
        if (i % 100000 == 0) cout << "Reads " << i / 100000 << " samples of " << samplesCount / 100000 << " (x100000)" << endl;

        Sample s;
        readSample(is, s);

        updateConvergence(convergence, s);
        updateLines(imgLines, s);
        updateDistributions(distributions, prRecognize, recognizeSum, s);
        updatePositioning(positioning, s);
    }
    is.close();

    vector<vector<double>> prRecognizeNorm = getNormPrRecognize(prRecognize, recognizeSum);

    vector<size_t> probabilityGood(PROBABILITY_GOOD_RESOLUTION);
    long double sumPr = 0;

    size_t groupCount;
    is = openToRead(FILE_SAMPLES_GROUP_NAME, groupCount);
    for (size_t i = 0; i < groupCount; i++)
    {
        if (i % 100 == 0) cout << "Reads " << i / 100 << " group of " << groupCount / 100 << " (x100)" << endl;

        vector<Sample> group;
        readGroup(is, group);

        sumPr += updateProbabilityGood(probabilityGood, group);
    }
    is.close();

    double prGood = sumPr / groupCount;

    drawConvergence(convergence, prGood);
    drawLines(imgLines);
    drawDistributions(distributions, prRecognizeNorm);
    drawPositioning(positioning);
    drawProbabilityGood(probabilityGood, prGood);

    cv::waitKey();

    return 0;
}