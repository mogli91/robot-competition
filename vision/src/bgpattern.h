#ifndef BGPATTERN_H
#define BGPATTERN_H

#include "opencv2/core/core.hpp"
#include "opencv2/opencv.hpp"
#include <ostream>
#include <cmath>

// locals
#include "defines.h"

class BGPattern {
private:
	static int patterncount;
	double m_dist_th;     // defines the radius of this cluster
	double m_curr[3];     // stores the mean GBR of bgpattern in previous frame
	double m_acc[3];  // accumulates GBR values of colors close to previous mean
	int m_count;          // counts the number of accumulated pixels
	int m_id;
	cv::Mat m_mask;		// every pattern has its own mask image

public:
	static void resetPatternCount() {
		patterncount = 0;
	}
	;

	// default constructor
	BGPattern() {
		m_curr[0] = m_curr[1] = m_curr[2] = 0.0;
		m_acc[0] = m_acc[1] = m_acc[2] = 0.0;
		m_count = 0;
		m_id = patterncount;
		m_dist_th = 50;
		++patterncount;
	}

	// constructor with initial color guess (GBR)
	BGPattern(double, double, double, int, int);

	// constructor with initial color guess (GBR) and th
	BGPattern(double, double, double, double, int, int);

	// destructor
	~BGPattern() {
	}
	;

	void setDistTh(double th);

	void reset();

	bool addPixel(uchar *gbr, int idx);

    bool addPixel(double *gbr, int idx);
    
	double computeMean();

	void getCurrMean(double *mean);

	double dist(uchar* pixel);

	double dist(double* pixel);

	void maskImage(cv::Mat src, cv::Mat dst);
    
    bool inRange(double);
    
    cv::Mat getMask() {return m_mask;};

	friend std::ostream& operator<<(std::ostream &out, BGPattern &pattern);

};
#endif
