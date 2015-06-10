#ifndef RANGEFINDER_H
#define RANGEFINDER_H

#include "opencv2/core/core.hpp"
#include "opencv2/opencv.hpp"
#include <ostream>
#include <cmath>
#include <vector>
#include <cmath>

// locals
#include "defines.h"
#include "Masks.h"

using namespace cv;
using namespace std;

struct CornerBeacon {
    Point position;
    int type;
};

struct RegressionLine {
    float delta_x;
    float delta_y;
    int intercept;
    float error;
};

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
    
    // 3D measurements
    vector<double> m_lateral_offset_cm_per_px;
    double m_distance_cm_per_px;
    
    Mat m_mask;		// every pattern has its own mask image
    Mat m_integral;
    
    Point m_beacon;
    Vec4f m_line;
    double m_error;
    
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
    
    bool findBrush(const cv::Mat &img_integral, Rect &brush);
    
    int determineTerrain(const cv::Mat &img_integral, Rect &brush);
    
    // in world coordinates
    
    void getBottleCoordinates(vector<Point> &dst);
    
    void getRayHeights(vector<int> &dst);
    
    // in px coordinates
    void getBottles(vector<Point> &dst);
    void getRays(vector<Point> &dst);
    void getBeacon(Point &dst);
    void getLineParameters(float &dx, float &dy, int &intercept, float &error);
    void getLineParameters(RegressionLine &line);
    
    double fitTerrainLine(Vec4f &v);
    
    
    int findBeacon(cv::Rect &roi);
    
    cv::Mat getMask() {return m_mask;};
    
    Point getWorldCoordinates(const cv:: Point &p_img);
    
    void extendBehindBottles();
    
private:
    void initScales();
    
};
#endif
