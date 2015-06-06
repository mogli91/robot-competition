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
public:
    double dist(double *a, double *b);
    cv::Rect getROI() {return m_roi;};
    
    virtual bool match(cv::Mat img_integral, cv::Rect roi, double threshold) = 0;
    virtual void computeMeanInner(cv::Mat img_integral, cv::Rect roi, double* dst) = 0;
};

class BottleFar : public Mask {
private:
    int m_blocksize;
    double m_mean[3];
    double m_compare[3];
public:
    BottleFar(int blocksize);
    bool match(cv::Mat img_integral, cv::Rect roi, double threshold);
    void computeMeanInner(cv::Mat img_integral, cv::Rect roi, double* dst);
};

#endif // MASKS_H