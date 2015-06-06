#ifndef RANGEFINDER_H
#define RANGEFINDER_H

#include "opencv2/core/core.hpp"
#include "opencv2/opencv.hpp"
#include <ostream>
#include <cmath>
#include <vector>

// locals
#include "defines.h"
#include "Masks.h"

using namespace cv;
using namespace std;

class RangeFinder {
private:
    int m_height;
    int m_width;
    int m_blocksize;
    int m_numRays;
    int m_offset;
    int m_step;
    double m_dist_th;
    vector<Rect> m_rays;
    vector<Rect> m_bottles;
    
    Mat m_mask;		// every pattern has its own mask image
    Mat m_integral;
    
public:
    // default constructor
    RangeFinder(int height, int width, int blocksize, double th, int offset = 0);
    
    // destructor
    ~RangeFinder() {
    }
    ;
    
    void reset();
    
    void computeMean(Rect, double*);
    
    double dist(double* a, double *b);
    
    void rollOut(Mat src, Mat dst);
    
    void drawMask(Mat dst);
    
    bool inRange(double);
    
    void locateBottles();
    
    void getBottleCoordinates(vector<Point> &dst);
    
    cv::Mat getMask() {return m_mask;};
    
};
#endif
