#ifndef MASKS_H
#define MASKS_H

#include "opencv2/core/core.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <iomanip>
#include <vector>

// local files
#include "defines.h"

using namespace cv;

class Mask {
protected:
    cv::Rect m_roi;
    int m_blocksize;
    double m_mean[3];
    double m_compare[3];
public:
    
    enum TYPE {FAR, CLOSE_UPRIGHT, CLOSE_FLAT, CLOSE_45P, CLOSE_45N,
        VERY_CLOSE_UPRIGHT, VERY_CLOSE_FLAT, VERY_CLOSE_45P, VERY_CLOSE_45N};
    
    Mask(int blocksize, int type);
    
    double dist(double *a, double *b);
    cv::Rect getROI() {return m_roi;};
    
    virtual bool match(const cv::Mat &img_integral, cv::Point p, double threshold) = 0;
    void computeMeanInnerRect(const cv::Mat &img_integral, cv::Rect roi, double* dst);
    bool matchRect(const cv::Mat &img_integral, cv::Point p, double threshold);
};

class BottleFar : public Mask {
public:
    BottleFar(int blocksize) : Mask(blocksize, FAR){};
    bool match(const cv::Mat &img_integral, cv::Point p, double threshold) {
        return matchRect(img_integral, p, threshold);
    };
};

class BottleCloseUpright : public Mask {
public:
    BottleCloseUpright(int blocksize) : Mask(blocksize, CLOSE_UPRIGHT){};
    bool match(const cv::Mat &img_integral, cv::Point p, double threshold) {
        return matchRect(img_integral, p, threshold);
    };
};

class BottleCloseFlat : public Mask {
public:
    BottleCloseFlat(int blocksize) : Mask(blocksize, CLOSE_FLAT){};
    bool match(const cv::Mat &img_integral, cv::Point p, double threshold) {
        return matchRect(img_integral, p, threshold);
    };
};

class BottleVeryCloseUpright : public Mask {
public:
    BottleVeryCloseUpright(int blocksize) : Mask(blocksize, VERY_CLOSE_UPRIGHT){};
    bool match(const cv::Mat &img_integral, cv::Point p, double threshold) {
        return matchRect(img_integral, p, threshold);
    };
};

class BottleVeryCloseFlat : public Mask {
public:
    BottleVeryCloseFlat(int blocksize) : Mask(blocksize, VERY_CLOSE_FLAT){};
    bool match(const cv::Mat &img_integral, cv::Point p, double threshold) {
        return matchRect(img_integral, p, threshold);
    };
};


//class BottleCloseUpright : public Mask {
//public:
//    BottleCloseUpright(int blocksize);
//    bool match(cv::Mat img_integral, cv::Point p, double threshold);
//};

#endif // MASKS_H