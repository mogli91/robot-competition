#ifndef DETECTOR_H
#define DETECTOR_H
#include "opencv2/core/core.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <iomanip>
#include <vector>
#include <pthread.h>

// local files
#include "bgpattern.h"
#include "defines.h"
#include "RangeFinder.h"

using namespace std;
using namespace cv;

struct RegressionLine {
    float delta_x;
    float delta_y;
    int intercept;
    float error;
};

struct VisionMeasure {
    vector<Point> bottles;
    vector<Point> rays; //between 20 and 250 cm
    RegressionLine line;
    Point beacon;
};

class Detector {
private:
	VideoCapture cap;
    Mat frame;
    Mat img_blur, mask, cross3;
	Size k_blur;
	int f_height, f_width;
	float exposure;

    bool normalize;
	// initial color guess
//	const double color_tiles[3] = {114.30, 121.76, 118.67};
//    const double color_wood[3] = {154.6154, 169.2896, 170.7563};
//    const double color_grass[3] = { 44.09, 58.91, 51.16};
//    const double color_obstacle[3] = { 169.7336, 150.2975, 131.2219};
//
//    const double n_color_tiles[3] = {0.5546, 0.5954, 0.5813};
//    const double n_color_wood[3] = { 0.5409,   0.5922, 0.5973    };
//    const double n_color_grass[3] = {0.4808, 0.6400, 0.5994};
//    const double n_color_obstacle[3] = { 0.6480,    0.5738 , 0.5009    };
//    
//	// patterns to look for;
//	BGPattern *bg_tile = NULL;
//	BGPattern *bg_wood = NULL;
//	BGPattern *bg_grass = NULL;
//	BGPattern *bg_obstacle = NULL;

	vector<BGPattern*> patterns;
    
    RangeFinder *m_rangeFinder = NULL;
    
    VisionMeasure m_measure;

public:
//	Detector(Mat &frame, int camnum = -1, float exposure = 0.05, int f_height = 240, int f_width = 320);
    Detector(int camnum = -1, float exposure = 0.05, int f_height = 240, int f_width = 320);
    Detector(const string &filename);
	~Detector();
	void detect();
    void printSettings();
    Mat getFrame();
    Mat getMask();
    bool isReady();
    void getMeasurement(VisionMeasure &vm);

    
private:
    void maskImage(cv::Mat src, cv::Mat dst);
    void normalize_px(uchar*, double*);
    void findRanges(Mat img);
    void kMeansSegmentation(Mat img);
    void setSettings();
    void computeMeasurement();
};
#endif // DETECTOR_H
