#include "RangeFinder.h"

RangeFinder::RangeFinder(int height, int width, int blocksize, double th, int offset) {
    m_height = height;
    m_width = width;
    m_blocksize = blocksize;
    m_numRays = width / blocksize;
    m_dist_th = th;
    m_offset = offset;
    m_step = blocksize / 2;
    
    // start rays at bottom of image
    for (int r = 0; r < m_numRays; ++r) {
        m_rays.push_back(Rect(r * blocksize, m_height - m_blocksize - m_offset, m_blocksize, m_blocksize + m_offset));
    }
    
    m_integral = Mat::zeros(m_height, m_width, CV_32SC3);
    m_mask = Mat::zeros(m_height, m_width, CV_8UC1);
}

void RangeFinder::reset() {
    m_integral = Scalar(0);
    m_mask = Scalar(0);
    for (int r = 0; r < m_numRays; ++r) {
        m_rays[r].height = m_blocksize + m_offset;
        m_rays[r].y = m_height - m_blocksize - m_offset;
    }
}

void RangeFinder::computeMean(Rect rect, double *dst) {
    int col0 = rect.x;
    int row0 = rect.y;
    
    int col1 = rect.x + rect.width;
    int row1 = rect.y + rect.height;
    
    int area = (row1 - row0) * (col1 - col0);
    
    assert(row0 >= 0 && col0 >= 0 && row1 < m_integral.rows && col1 < m_integral.cols);
    int *sumxy = (int*)m_integral.data;
    int idx_r0_c0 = (row0 * m_integral.cols + col0) * 3;
    int idx_r1_c0 = (row1 * m_integral.cols + col0) * 3;
    int idx_r0_c1 = (row0 * m_integral.cols + col1) * 3;
    int idx_r1_c1 = (row1 * m_integral.cols + col1) * 3;
    
    for (int color = 0; color < 3; ++color) {
        dst[color] = (double)((sumxy[idx_r0_c0 + color] + sumxy[idx_r1_c1 + color]
                              - sumxy[idx_r0_c1 + color] - sumxy[idx_r1_c0 + color]));
    }
    for (int color = 0; color < 3; ++color) {
        dst[color] /= area;
    }
}

double RangeFinder::dist(double* a, double *b) {
    double dist = sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) + (a[2] - b[2]) * (a[2] - b[2]));
    //        std::cout << dist << std::endl;
    return dist;
}

void RangeFinder::rollOut(cv::Mat src, cv::Mat dst) {
    // accept only char type matrices
    assert(src.depth() == CV_8U && dst.depth() == CV_8U);
    assert(src.channels() == 3 && dst.channels() == 1);
    assert(src.rows == dst.rows && src.cols == dst.cols);
    
    reset();
    
    integral(src, m_integral);
    double mu_new[3] = {0.0, 0.0, 0.0};
    double mu_old[3] = {0.0, 0.0, 0.0};
    double color_dist;
    Rect r_tmp;
    
    for (int r = 0; r < m_numRays; ++r) {
        r_tmp = Rect(m_rays[r].x, m_rays[r].y, m_blocksize, m_blocksize);
        computeMean(r_tmp, mu_old);
        cout << r << ": ";
        // walk until we hit something (color change)
        int lower_bottom = m_rays[r].y;
        for (int d = lower_bottom; d >= m_step; d -= m_step) {
            r_tmp = Rect(r * m_blocksize, d - m_step, m_blocksize, m_blocksize);
            computeMean(r_tmp, mu_new);
            color_dist = dist(mu_new, mu_old);
            cout << color_dist << ", ";
            if (color_dist < m_dist_th) {
                m_rays[r].height += m_step; // make this ray longer by 1*blocksize
                m_rays[r].y = d - m_step;
                memcpy(mu_old, mu_new, 3 * sizeof(double));
            } else {
                 break;
            }
        }
        cout << endl;
    }
    
    locateBottles();
    
    for (int b = 0; b < m_bottles.size(); ++b) {
        rectangle(src, m_bottles[b], Scalar(0, 255, 0));
    }
    
    drawMask(dst);
    
}

void RangeFinder::locateBottles() {
    int bs = m_blocksize;
    BottleFar bottleFar = BottleFar(bs);
    BottleCloseUpright bottleCU = BottleCloseUpright(bs);
    BottleCloseFlat bottleCF = BottleCloseFlat(bs);
    BottleClose45P bottleC45P = BottleClose45P(bs);
    BottleClose45N bottleC45N = BottleClose45N(bs);
    BottleVeryCloseUpright bottleVCU = BottleVeryCloseUpright(bs);
    BottleVeryCloseFlat bottleVCF = BottleVeryCloseFlat(bs);
    BottleVeryClose45P bottleVC45P = BottleVeryClose45P(bs);
    BottleVeryClose45N bottleVC45N = BottleVeryClose45N(bs);
    double threshold = 30;
    Point p;
    
    m_bottles.clear();
    
    for (int r = 0; r < m_numRays; ++r) {
        p.x = m_rays[r].x;
        p.y = m_rays[r].y - bs;
        if (bottleFar.match(m_integral, p, threshold)) {
            m_bottles.push_back(bottleFar.getROI());
        }
        p.y = m_rays[r].y - bs;
        if (bottleCU.match(m_integral, p, threshold)) {
            m_bottles.push_back(bottleCU.getROI());
        }
        if (bottleCF.match(m_integral, p, threshold)) {
            m_bottles.push_back(bottleCF.getROI());
        }
//        if (bottleC45P.match(m_integral, p, threshold)) {
//            m_bottles.push_back(bottleC45P.getROI());
//        }
//        if (bottleC45N.match(m_integral, p, threshold)) {
//            m_bottles.push_back(bottleC45N.getROI());
//        }
        
        p.y = m_rays[r].y - 1.5 * bs;
        if (bottleVCU.match(m_integral, p, threshold)) {
            m_bottles.push_back(bottleVCU.getROI());
        }
        if (bottleVCF.match(m_integral, p, threshold)) {
            m_bottles.push_back(bottleVCF.getROI());
        }
//        if (bottleVC45P.match(m_integral, p, threshold)) {
//            m_bottles.push_back(bottleVC45P.getROI());
//        }
//        if (bottleVC45N.match(m_integral, p, threshold)) {
//            m_bottles.push_back(bottleVC45N.getROI());
//        }
        
    }
}

void RangeFinder::drawMask(Mat dst) {
    Mat tmp;
    for (int r = 0; r < m_numRays; ++r) {
        tmp = Mat(m_mask, m_rays[r]);
        tmp = Scalar(1);
    }
    m_mask.copyTo(dst);
}

void RangeFinder::getBottleCoordinates(vector<Point> &dst) {
//    int x, y, x_world, y_world;
//    dst.clear();
//    for (vector<Rect>::iterator it = m_bottles.begin(); it != m_bottles.end(); ++it) {
//        x = it->x + it->width / 2;  // want center of bottle
//        y = it->y + it->height;     // want closest point of bottle
//        
//        y_world = VISION_DIST_BOTTOM + (m_height - y - 1) * m_height / (VISION_DIST_TOP - VISION_DIST_BOTTOM);
//        
//        // TODO: proper x coordinate
//        if (x < m_width / 3) {
//            x_world = -1;
//        } else if (x < 2 * m_width / 3) {
//            x_world = 0;
//        } else {
//            x_world = 1;
//        }
//        
//        dst.push_back(Point(x_world, y_world));
//    }
}
