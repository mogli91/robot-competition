#include "Masks.h"

Mask::Mask(int blocksize, int type) {
    m_blocksize = blocksize;
    
    switch (type) {
        case FAR:
            m_roi.height = m_blocksize;
            m_roi.width = m_blocksize;
            break;
        case CLOSE_UPRIGHT:
            m_roi.height = (m_blocksize * 3) / 2;
            m_roi.width = m_blocksize;
            break;
        case CLOSE_FLAT:
            m_roi.height = m_blocksize;
            m_roi.width = (m_blocksize * 3) / 2;
        case CLOSE_45P:
            m_roi.height = (m_blocksize * 3) / 2;
            m_roi.width = (m_blocksize * 3) / 2;
            break;
        case CLOSE_45N:
            m_roi.height = (m_blocksize * 3) / 2;
            m_roi.width = (m_blocksize * 3) / 2;
            break;
        case VERY_CLOSE_UPRIGHT:
            m_roi.height = m_blocksize * 2;
            m_roi.width = m_blocksize;
            break;
        case VERY_CLOSE_FLAT:
            m_roi.height = m_blocksize;
            m_roi.width = m_blocksize * 2;
        case VERY_CLOSE_45P:
            m_roi.height = (m_blocksize * 3) / 2;
            m_roi.width = m_blocksize * 2;
            break;
        case VERY_CLOSE_45N:
            m_roi.height = (m_blocksize * 3) / 2;
            m_roi.width = m_blocksize * 2;
            break;
        default:
            break;
    }
    
}

double Mask::dist(double* a, double *b) {
    double dist = sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) + (a[2] - b[2]) * (a[2] - b[2]));
            std::cout << dist << std::endl;
    return dist;
}

void Mask::computeMeanInnerRect(const cv::Mat &img_integral, cv::Rect roi, double *dst) {
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

bool Mask::matchRect(const cv::Mat &img_integral, cv::Point p, double threshold) {
    if ((p.x - m_blocksize / 2) < 0
        || (p.y - m_blocksize / 2) < 0
        || (p.y + m_roi.height + m_blocksize / 2) >= img_integral.rows
        || (p.x + m_roi.width + m_blocksize / 2) >= img_integral.cols) {
        
        return false;
    }
    
    m_roi.x = p.x;
    m_roi.y = p.y;
    computeMeanInnerRect(img_integral, m_roi, m_mean);
    
    Rect tmp;
    double color_dist;
    
    // left
    tmp.x = m_roi.x - m_blocksize/ 2; tmp.y = m_roi.y; tmp.width = m_blocksize / 2; tmp.height = m_blocksize;
    computeMeanInnerRect(img_integral, tmp, m_compare);
    
    color_dist = dist(m_compare, m_mean);
    
    if (color_dist < threshold) {
        return false;
    }
    
    // right
    tmp.x = m_roi.x + m_roi.width; tmp.y = m_roi.y; tmp.width = m_blocksize / 2; tmp.height = m_blocksize;
    computeMeanInnerRect(img_integral, tmp, m_compare);
    
    color_dist = dist(m_compare, m_mean);
    
    if (color_dist < threshold) {
        return false;
    }
    
    // top
    tmp.x = m_roi.x; tmp.y = m_roi.y - m_blocksize / 2; tmp.width = m_blocksize; tmp.height = m_blocksize / 2;
    computeMeanInnerRect(img_integral, tmp, m_compare);
    
    color_dist = dist(m_compare, m_mean);
    
    if (color_dist < threshold) {
        return false;
    }
    
    // bottom
    tmp.x = m_roi.x; tmp.y = m_roi.y + m_roi.height; tmp.width = m_blocksize; tmp.height = m_blocksize / 2;
    computeMeanInnerRect(img_integral, tmp, m_compare);
    
    color_dist = dist(m_compare, m_mean);
    
    if (color_dist < threshold) {
        return false;
    }
    
    return true;
    
}


//// ---------------------- BOTTLE FAR AWAY ----------------------------
//
//bool BottleFar::match(const cv::Mat &img_integral, cv::Point p, double threshold) {
//    if ((p.x - m_roi.height - m_blocksize / 2) < 0
//        || (p.y - m_blocksize / 2) < 0
//        || (p.y + m_blocksize / 2) >= img_integral.rows
//        || (p.x + m_roi.width + m_blocksize / 2) >= img_integral.cols) {
//        
//        return false;
//    }
//    
//    return matchRect(img_integral, p, threshold);
//}

// ------------------------- BOTTLE CLOSE ------------------------

