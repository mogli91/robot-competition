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
    int m_area;
    int m_blocksize;
    double m_mean[3];
    double m_compare[3];
    double m_sum[3];
public:
    
    enum TYPE {FAR, CLOSE_UPRIGHT, CLOSE_FLAT, CLOSE_45P, CLOSE_45N,
        VERY_CLOSE_UPRIGHT, VERY_CLOSE_FLAT, VERY_CLOSE_45P, VERY_CLOSE_45N, BEACON};
    
    Mask(int blocksize, int type);
    
    double dist(double *a, double *b);
    void computeSum(const cv::Mat &img_integral, cv::Rect roi, int *dst);
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

class BottleClose45P : public Mask {
public:
    BottleClose45P(int blocksize) : Mask(blocksize, CLOSE_45P){};
    bool match(const cv::Mat &img_integral, cv::Point p, double threshold);
};

class BottleClose45N : public Mask {
public:
    BottleClose45N(int blocksize) : Mask(blocksize, CLOSE_45N){};
    bool match(const cv::Mat &img_integral, cv::Point p, double threshold);
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

class BottleVeryClose45P : public Mask {
public:
    BottleVeryClose45P(int blocksize) : Mask(blocksize, VERY_CLOSE_45P){};
    bool match(const cv::Mat &img_integral, cv::Point p, double threshold);
};

class BottleVeryClose45N : public Mask {
public:
    BottleVeryClose45N(int blocksize) : Mask(blocksize, VERY_CLOSE_45N){};
    bool match(const cv::Mat &img_integral, cv::Point p, double threshold);
};

class Beacon : public Mask {
private:
    double *m_color[4];
    double m_red[3] = {0, 0, 255};
    double m_blue[3] = {255, 102, 153};
    double m_green[3] = {255, 0, 0};
    double m_yellow[3] = {255, 255, 0};
    int m_corner;
public:
    enum CORNER {RED, BLUE, GREEN, YELLOW, NONE};
    Beacon(int blocksize) : Mask(blocksize, BEACON){
        m_color[RED] = m_red;
        m_color[BLUE] = m_blue;
        m_color[GREEN] = m_green;
        m_color[YELLOW] = m_yellow;
        m_corner = NONE;
    };
    bool match(const cv::Mat &img_integral, cv::Point p, double threshold);
    bool matchGray(const cv::Mat &img_integral, cv::Point p, double threshold);
//    void setColor(double *color) { memcpy(m_color, color, sizeof(double)); };
    int getCorner() {return m_corner;};
};

#endif // MASKS_H
