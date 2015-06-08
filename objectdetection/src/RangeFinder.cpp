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
    
    initScales();
}

void RangeFinder::initScales() {
    m_lateral_offset_cm_per_px = vector<double>(m_height);
    
    // scaling factors for 3d measurements
    m_distance_cm_per_px = (VISION_DIST_TOP - VISION_DIST_BOTTOM) / (1.0 * m_height);
    double cm_per_px_bottom = VISION_CM_BOTTOM / (1.0 * m_width);
    double cm_per_px_top = VISION_CM_TOP / (1.0 * m_width);
    double cm_per_px_at_row;
    for (int i = 0; i < m_height; ++i) {
        cm_per_px_at_row = cm_per_px_top + (cm_per_px_bottom - cm_per_px_top) / m_height * i;
        m_lateral_offset_cm_per_px[i] = cm_per_px_at_row;
    }
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
//        cout << r << ": ";
        // walk until we hit something (color change)
        int lower_bottom = m_rays[r].y;
        for (int d = lower_bottom; d >= m_step; d -= m_step) {
            r_tmp = Rect(r * m_blocksize, d - m_step, m_blocksize, m_blocksize);
            computeMean(r_tmp, mu_new);
            color_dist = dist(mu_new, mu_old);
//            cout << color_dist << ", ";
            if (color_dist < m_dist_th) {
                m_rays[r].height += m_step; // make this ray longer by 1*blocksize
                m_rays[r].y = d - m_step;
                memcpy(mu_old, mu_new, 3 * sizeof(double));
            } else {
                 break;
            }
        }
//        cout << endl;
    }
    
    locateBottles();
    
    for (int b = 0; b < m_bottles.size(); ++b) {
        rectangle(src, m_bottles[b], Scalar(0, 255, 0));
    }
    Rect tmp;
    int beacon = findBeacon(tmp);
    if(beacon) {
        rectangle(src, tmp, Scalar(255, 0, 255));
        m_beacon.x = tmp.x + tmp.width/2;
        m_beacon.y = m_height - (tmp.y + tmp.height);
    } else {
        m_beacon.x = -1;
        m_beacon.y = -1;
    }
    
    
    m_error = fitTerrainLine(m_line);
    line(src, Point(m_line[2] - m_line[0]*1000,m_line[3] - m_line[1]*1000),Point(m_line[2] + m_line[0]*1000, m_line[3]+m_line[1]*1000), Scalar(255,255,0));
    
//    std::cout << "error " << m_error << std::endl;
    
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
    bool found = false;
    
    for (int r = 0; r < m_numRays; ++r) {
        found = false;
        for (int run = 0; run < 2 && !found; ++run) {
            p.x = m_rays[r].x;
            p.y = m_rays[r].y - bs - bs / 2 * run;
            
            if (bottleFar.match(m_integral, p, threshold)) {
                m_bottles.push_back(bottleFar.getROI());
                found = true;
                break;
            }
        
            // apply big mask only when we are close by
            if (p.y > m_height * 0.5) {
                if (bottleCU.match(m_integral, p, threshold)) {
                    m_bottles.push_back(bottleCU.getROI());
                    found = true;
                    break;
                }
                if (bottleCF.match(m_integral, p, threshold)) {
                    m_bottles.push_back(bottleCF.getROI());
                    found = true;
                    break;
                }
                
                
                p.y = m_rays[r].y - 1.5 * bs - run * bs/2;
                
                if (bottleC45P.match(m_integral, p, threshold)) {
                    m_bottles.push_back(bottleC45P.getROI());
                    found = true;
                    break;
                }
                if (bottleC45N.match(m_integral, p, threshold)) {
                    m_bottles.push_back(bottleC45N.getROI());
                    found = true;
                    break;
                }
                
                if (bottleVCU.match(m_integral, p, threshold)) {
                    m_bottles.push_back(bottleVCU.getROI());
                    found = true;
                    break;
                }
                if (bottleVCF.match(m_integral, p, threshold)) {
                    m_bottles.push_back(bottleVCF.getROI());
                    found = true;
                    break;
                }
                if (bottleVC45P.match(m_integral, p, threshold)) {
                    m_bottles.push_back(bottleVC45P.getROI());
                    found = true;
                    break;
                }
                if (bottleVC45N.match(m_integral, p, threshold)) {
                    m_bottles.push_back(bottleVC45N.getROI());
                    found = true;
                    break;
                }
            }
        }
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
    // TODO sort bottles by distance
    int x, y;
    dst.clear();
    for (vector<Rect>::iterator it = m_bottles.begin(); it != m_bottles.end(); ++it) {
        x = it->x + it->width / 2;  // want center of bottle
        y = it->y + it->height;     // want closest point of bottle
        
//        angle = (atan2(lateral_offset_cm, distance_cm) * 180) / PI;
//        dst.push_back(getWorldCoordinates(Point(x, y)));
        dst.push_back(Point(x, y));
    }
}

void RangeFinder::getRayHeights(vector<int> &dst) {
    int y, distance_cm;
    dst.clear();
    int unused = 3;
    for (int i = 0 + unused; i < m_numRays - unused; ++i) {
        y = m_rays[i].y;
        
        distance_cm = VISION_DIST_BOTTOM + m_distance_cm_per_px * (m_height - y);
        dst.push_back(distance_cm);
    }
}

int RangeFinder::findBeacon(cv::Rect &roi) {
    Beacon b = Beacon(m_blocksize/2);
//    double color_th = 300;
    double gray_th = 30;
    
    Point p(m_blocksize/2,0);
    
    if (b.matchGray(m_integral, p, gray_th)) {
//    if (b.matchGray(m_integral, p, color_th)) {
        roi = b.getROI();
        return 1;
    }
    else
        return 0;
}

Point RangeFinder::getWorldCoordinates(const cv:: Point &p_img) {
    int distance_cm;
    int lateral_offset_cm;
    
    distance_cm = VISION_DIST_BOTTOM + m_distance_cm_per_px * (m_height - p_img.y);
    lateral_offset_cm = m_lateral_offset_cm_per_px[p_img.y] * (p_img.x - m_width / 2.0);
    
    return Point(lateral_offset_cm, distance_cm);
}

// in px coordinates
void RangeFinder::getBottles(vector<Point> &dst) {
    dst.clear();
    Rect tmp;
    for (int i = 0; i < m_bottles.size(); ++i) {
        tmp = m_bottles[i];
        dst.push_back(Point((tmp.x + tmp.width/2) - m_width, m_height - (tmp.y + tmp.height)));
    }
}
void RangeFinder::getRays(vector<Point> &dst) {
    dst.clear();
    Rect tmp;
    for (int i = 0; i < m_rays.size(); ++i) {
        tmp = m_rays[i];
        dst.push_back(Point((tmp.x + tmp.width/2) - m_width, m_height - (tmp.y + tmp.height)));
    }
}
void RangeFinder::getBeacon(Point &dst) {
    dst.x = m_beacon.x - m_width/2;
    dst.y = m_height - m_beacon.y;
}

double RangeFinder::fitTerrainLine(Vec4f &line) {
    vector<Point> pts;
    Rect tmp;
    for (int i = 3; i < m_rays.size() - 3; ++i) {
        tmp = m_rays[i];
        pts.push_back(Point2f(tmp.x + tmp.width/2, m_height - tmp.height));
    }
    
    fitLine(pts, line, CV_DIST_L1, 0, 0.01, 0.01);
    
    if (line[0] == 0) {
        return -1;
    }
    float slope = line[1] / line[0];
    float x0 = line[2];
    float y0 = line[3];
    
    float x, y; // = y0 - (x0-xb)*vy/vx;//=y0 - x0/vx*vy;// = y0 - a*vy //
    
    float sum = 0;
    for(uint i = 0; i < pts.size(); i++)
    {
        x = pts[i].x;
        y = y0 + (x - x0) * slope;
//        sum += abs(pts[i].y - y);
        sum += (pts[i].y - y) * (pts[i].y - y);
    }
    return sum/m_numRays;
}

void RangeFinder::getLineParameters(float &dx, float &dy, int &intercept, float &error) {
    float slope = m_line[1] / m_line[0];
    float x0 = m_line[2];
    float y0 = m_line[3];
    float x = m_width / 2;
    
    dx = m_line[0];
    dy = - m_line[1];
    
//    std::cout << (dx - m_line[0]) << std::endl;
    
    intercept = m_height - (y0 + (x - x0) * slope);
    error = m_error;
}

void RangeFinder::getLineParameters(RegressionLine &line) {
    float slope = m_line[1] / m_line[0];
    float x0 = m_line[2];
    float y0 = m_line[3];
    float x = m_width / 2;
    
    line.delta_x = m_line[0];
    line.delta_y = - m_line[1];
    
//    std::cout << (line.delta_x - m_line[0]) << std::endl;
    
    line.intercept = m_height - (y0 + (x - x0) * slope);
    line.error = m_error;
}
