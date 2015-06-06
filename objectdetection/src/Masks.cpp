#include "Masks.h"

BottleFar::BottleFar(int blocksize) {
    m_blocksize = blocksize;
    m_roi.height = m_blocksize;
    m_roi.width = m_blocksize;
}

void BottleFar::computeMeanInner(cv::Mat img_integral, cv::Rect roi, double *dst) {
    int col0 = roi.x;
    int row0 = roi.y;
    
    int col1 = roi.x + roi.width;
    int row1 = roi.y + roi.height;
    
    int area = (row1 - row0) * (col1 - col0);
    
    assert(row0 >= 0 && col0 >= 0 && row1 < img_integral.rows && col1 < img_integral.cols);
    int *sumxy = (int*)img_integral.data;
    int idx_r0_c0 = (row0 * img_integral.cols + col0) * 3;
    int idx_r1_c0 = (row1 * img_integral.cols + col0) * 3;
    int idx_r0_c1 = (row0 * img_integral.cols + col1) * 3;
    int idx_r1_c1 = (row1 * img_integral.cols + col1) * 3;
    
    for (int color = 0; color < 3; ++color) {
        dst[color] = (double)((sumxy[idx_r0_c0 + color] + sumxy[idx_r1_c1 + color]
                               - sumxy[idx_r0_c1 + color] - sumxy[idx_r1_c0 + color]));
    }
    for (int color = 0; color < 3; ++color) {
        dst[color] /= area;
    }
}

bool BottleFar::match(cv::Mat img_integral, cv::Rect roi, double threshold) {
    if ((roi.x - m_blocksize / 2) < 0
        || (roi.y - m_blocksize / 2) < 0
        || (roi.y + roi.height + m_blocksize / 2) >= img_integral.rows
        || (roi.x + roi.width + m_blocksize / 2) >= img_integral.cols) {
        
        return false;
    }
    
    roi.height = m_blocksize;
    roi.width = m_blocksize;
    computeMeanInner(img_integral, roi, m_mean);
    
    Rect tmp;
    double color_dist;
    
    // left
    tmp.x = roi.x - m_blocksize/ 2; tmp.y = roi.y; tmp.width = m_blocksize / 2; tmp.height = m_blocksize;
    computeMeanInner(img_integral, tmp, m_compare);
    
    color_dist = dist(m_compare, m_mean);
    
    if (color_dist < threshold) {
        return false;
    }
    
    // right
    tmp.x = roi.x + roi.width; tmp.y = roi.y; tmp.width = m_blocksize / 2; tmp.height = m_blocksize;
    computeMeanInner(img_integral, tmp, m_compare);
    
    color_dist = dist(m_compare, m_mean);
    
    if (color_dist < threshold) {
        return false;
    }
    
    // top
    tmp.x = roi.x; tmp.y = roi.y - m_blocksize / 2; tmp.width = m_blocksize; tmp.height = m_blocksize / 2;
    computeMeanInner(img_integral, tmp, m_compare);
    
    color_dist = dist(m_compare, m_mean);
    
    if (color_dist < threshold) {
        return false;
    }
    
    // bottom
    tmp.x = roi.x; tmp.y = roi.y + roi.height; tmp.width = m_blocksize; tmp.height = m_blocksize / 2;
    computeMeanInner(img_integral, tmp, m_compare);
    
    color_dist = dist(m_compare, m_mean);
    
    if (color_dist < threshold) {
        return false;
    }
    
    m_roi = Rect(roi);
    
    return true;
    
}

double Mask::dist(double* a, double *b) {
    double dist = sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) + (a[2] - b[2]) * (a[2] - b[2]));
    //        std::cout << dist << std::endl;
    return dist;
}