#include "opencv2/core/core.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <iomanip>
#include <cstdio>             /* standard I/O routines                      */
#include <unistd.h>
#include <cstring>
#include <cstdlib>
#include <ctime>

// locals
#include "defines.h"
#include "detector.h"

using namespace std;

//Mat frame;

static void help() {
	cout << "Usage: ./detect <name of image file>" << endl;
}

int main(int argc, char** args) {

    if (argc < 2) {
        help();
        return -1;
    }
    cout << "loading image " << args[1] << endl;

    Detector *detector;
    Mat image;
	
    if (0 == strcmp(args[1], "cam")) {
        detector = new Detector(-1, 0.2, 240, 320);
        image = Mat::zeros(240, 320, CV_8UC3);
    } else {
        image = cv::imread(args[1]);
        detector = new Detector(args[1]);
    }

    

    Mat cframe = Mat::zeros(image.rows, image.cols, CV_8UC3);
    Mat cframe_2 = Mat::zeros(image.rows, image.cols, CV_8UC3);
    Mat cmask = Mat::zeros(image.rows, image.cols, CV_8UC1);
    
    // for splitting
    Mat r = Mat::zeros(image.rows, image.cols, CV_8UC1); Mat r_masked = Mat::zeros(image.rows, image.cols, CV_8UC1);;
    Mat g = Mat::zeros(image.rows, image.cols, CV_8UC1); Mat g_masked = Mat::zeros(image.rows, image.cols, CV_8UC1);;
    Mat b = Mat::zeros(image.rows, image.cols, CV_8UC1); Mat b_masked = Mat::zeros(image.rows, image.cols, CV_8UC1);;
    Mat out[] = { b, g, r };
    Mat masked_in[] = { b_masked, g_masked, r_masked };
    int from_to[] = { 0,0, 1,1, 2,2 };
    
    // for displaying hsv
    Mat hsv = Mat::zeros(image.rows, image.cols, CV_8UC3);
    Mat h = Mat::zeros(image.rows, image.cols, CV_8UC1);
    Mat s = Mat::zeros(image.rows, image.cols, CV_8UC1);
    Mat v = Mat::zeros(image.rows, image.cols, CV_8UC1);
    Mat out_hsv[] = { h, s, v };
    
    // grayscale
    Mat gray = Mat::zeros(image.rows, image.cols, CV_8UC1);
    Mat th = Mat::zeros(image.rows, image.cols, CV_8UC1);
    Mat canny = Mat::zeros(image.rows, image.cols, CV_8UC1);
    
	for (;;) {
        detector->detect();
//        detector->findRanges(image);
        cframe = detector->getFrame().clone();
        cmask = detector->getMask().clone();
        
        r_masked = Scalar(0);
        g_masked = Scalar(0);
        b_masked = Scalar(0);
        // rgb
        mixChannels( &cframe, 1, out, 3, from_to, 3 );
        r.copyTo(r_masked, cmask);
        g.copyTo(g_masked, cmask);
        b.copyTo(b_masked, cmask);
        
        mixChannels( masked_in, 3, &cframe_2, 1, from_to, 3 );
        
//        // hsv
//        cvtColor(cframe_2, hsv, CV_BGR2HSV);
//        mixChannels( &hsv, 1, out_hsv, 3, from_to, 3 );
//        
//        // grayscale
//        cvtColor(cframe_2, gray, CV_BGR2GRAY);
//        threshold(gray, gray, 80, 255, THRESH_BINARY);
//        Canny(gray, canny, 100, 200);
//        adaptiveThreshold(gray, th, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 19, 0);
        
        
        imshow("preview", cframe);
        imshow("mask", cframe_2);
//        imshow("h", h);
//        imshow("s", s);
//        imshow("v", v);
//        imshow("gray", gray);
//        imshow("th", canny);
        char c = (char) waitKey(1);
        if (c == ' ') {
            break;
        }
    }
	return 0;
}

