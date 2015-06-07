#include "Masks.h"

Mask::Mask(int blocksize, int type) {
    m_blocksize = blocksize;
    
    switch (type) {
        case FAR:
            m_roi.height = m_blocksize;
            m_roi.width = m_blocksize;
            m_area = m_blocksize * m_blocksize;
            break;
        case CLOSE_UPRIGHT:
            m_roi.height = (m_blocksize * 3) / 2;
            m_roi.width = m_blocksize;
            m_area =  m_blocksize * m_blocksize * 3/ 2;
            break;
        case CLOSE_FLAT:
            m_roi.height = m_blocksize;
            m_roi.width = (m_blocksize * 3) / 2;
            m_area =  m_blocksize * m_blocksize * 3/ 2;
        case CLOSE_45P:
            m_roi.height = (m_blocksize * 3) / 2;
            m_roi.width = (m_blocksize * 3) / 2;
            m_area =  m_blocksize * m_blocksize * 7/ 4;
            break;
        case CLOSE_45N:
            m_roi.height = (m_blocksize * 3) / 2;
            m_roi.width = (m_blocksize * 3) / 2;
            m_area =  m_blocksize * m_blocksize * 7/ 4;
            break;
        case VERY_CLOSE_UPRIGHT:
            m_roi.height = m_blocksize * 2;
            m_roi.width = m_blocksize;
            m_area =  m_blocksize * m_blocksize * 2;
            break;
        case VERY_CLOSE_FLAT:
            m_roi.height = m_blocksize;
            m_roi.width = m_blocksize * 2;
            m_area =  m_blocksize * m_blocksize * 2;
        case VERY_CLOSE_45P:
            m_roi.height = (m_blocksize * 3) / 2;
            m_roi.width = m_blocksize * 2;
            m_area =  m_blocksize * m_blocksize * 5/ 2;
            break;
        case VERY_CLOSE_45N:
            m_roi.height = (m_blocksize * 3) / 2;
            m_roi.width = m_blocksize * 2;
            m_area =  m_blocksize * m_blocksize * 5/ 2;
            break;
        case BEACON:
            m_roi.width = m_blocksize;
            m_roi.height = 2 * m_blocksize;
        default:
            break;
    }
    
}

double Mask::dist(double* a, double *b) {
    double dist = sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) + (a[2] - b[2]) * (a[2] - b[2]));
//            std::cout << dist << std::endl;
    return dist;
}

void Mask::computeSum(const cv::Mat &img_integral, cv::Rect roi, int *dst) {
    if (roi.x < 0 || roi.y < 0 || (roi.x + roi.width) >= img_integral.cols ||  (roi.y + roi.height) >= img_integral.rows) {
        return;
    }
    int col0 = roi.x;
    int row0 = roi.y;
    
    int col1 = roi.x + roi.width;
    int row1 = roi.y + roi.height;
    
    int *sumxy = (int*)img_integral.data;
    int idx_r0_c0 = (row0 * img_integral.cols + col0) * 3;
    int idx_r1_c0 = (row1 * img_integral.cols + col0) * 3;
    int idx_r0_c1 = (row0 * img_integral.cols + col1) * 3;
    int idx_r1_c1 = (row1 * img_integral.cols + col1) * 3;
    
    for (int color = 0; color < 3; ++color) {
        dst[color] = (double)((sumxy[idx_r0_c0 + color] + sumxy[idx_r1_c1 + color]
                               - sumxy[idx_r0_c1 + color] - sumxy[idx_r1_c0 + color]));
    }

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


// ------------------------- BOTTLE CLOSE 45P ------------------------

bool BottleClose45P::match(const cv::Mat &img_integral, cv::Point p, double threshold) {
    if ((p.x - m_blocksize / 2) < 0
        || (p.y - m_blocksize / 2) < 0
        || (p.y + m_roi.height + m_blocksize / 2) >= img_integral.rows
        || (p.x + m_roi.width + m_blocksize / 2) >= img_integral.cols) {
        
        return false;
    }
    
    m_roi.x = p.x;
    m_roi.y = p.y;
    
    int tmp_sum1[3];
    int tmp_sum2[3];
    
    Rect lower_left(m_roi.x, m_roi.y + m_blocksize / 2, m_blocksize, m_blocksize);
    Rect upper_right(m_roi.x + m_blocksize / 2, m_roi.y , m_blocksize, m_blocksize);
    
    // sum up two blocks
    computeSum(img_integral, lower_left, tmp_sum1);
    computeSum(img_integral, upper_right, tmp_sum2);
    
    for (int i = 0; i < 3; ++i) {
        tmp_sum1[i] += tmp_sum2[i];
    }
    
    // subtract their intersection
    lower_left.x = m_roi.x + m_blocksize / 2;
    lower_left.y = m_roi.y + m_blocksize / 2;
    lower_left.width = m_blocksize / 2;
    lower_left.height = m_blocksize / 2;
    
    computeSum(img_integral, lower_left, tmp_sum2);
    for (int i = 0; i < 3; ++i) {
        tmp_sum1[i] -= tmp_sum2[i];
    }
    
    // compute mean
    for (int i = 0; i < 3; ++i) {
        m_mean[i] = (1.0 * tmp_sum1[i]) / m_area;
    }
    
    Rect tmp;
    double color_dist;
    
    // left
    tmp.x = m_roi.x - m_blocksize/ 2; tmp.y = m_roi.y + m_blocksize / 2; tmp.width = m_blocksize / 2; tmp.height = m_blocksize;
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
    tmp.x = m_roi.x + m_blocksize / 2; tmp.y = m_roi.y - m_blocksize / 2; tmp.width = m_blocksize; tmp.height = m_blocksize / 2;
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
    
    // little cube in the middle (left)
    tmp.x = m_roi.x; tmp.y = m_roi.y; tmp.width = m_blocksize / 2; tmp.height = m_blocksize / 2;
    computeMeanInnerRect(img_integral, tmp, m_compare);
    
    color_dist = dist(m_compare, m_mean);
    
    if (color_dist < threshold) {
        return false;
    }
    
    // little cube in the middle (right)
    tmp.x = m_roi.x + m_blocksize; tmp.y = m_roi.y + m_blocksize; tmp.width = m_blocksize / 2; tmp.height = m_blocksize / 2;
    computeMeanInnerRect(img_integral, tmp, m_compare);
    
    color_dist = dist(m_compare, m_mean);
    
    if (color_dist < threshold) {
        return false;
    }
    
    return true;
}

// ------------------------- BOTTLE CLOSE 45N ------------------------

bool BottleClose45N::match(const cv::Mat &img_integral, cv::Point p, double threshold) {
    if ((p.x - m_blocksize / 2) < 0
        || (p.y - m_blocksize / 2) < 0
        || (p.y + m_roi.height + m_blocksize / 2) >= img_integral.rows
        || (p.x + m_roi.width + m_blocksize / 2) >= img_integral.cols) {
        
        return false;
    }
    
    m_roi.x = p.x;
    m_roi.y = p.y;
    
    int tmp_sum1[3];
    int tmp_sum2[3];
    
    Rect upper_left(m_roi.x, m_roi.y, m_blocksize, m_blocksize);
    Rect lower_right(m_roi.x + m_blocksize / 2, m_roi.y + m_blocksize / 2 , m_blocksize, m_blocksize);
    
    // sum up two blocks
    computeSum(img_integral, upper_left, tmp_sum1);
    computeSum(img_integral, lower_right, tmp_sum2);
    
    for (int i = 0; i < 3; ++i) {
        tmp_sum1[i] += tmp_sum2[i];
    }
    
    // subtract their intersection
    upper_left.x = m_roi.x + m_blocksize / 2;
    upper_left.y = m_roi.y + m_blocksize / 2;
    upper_left.width = m_blocksize / 2;
    upper_left.height = m_blocksize / 2;
    
    computeSum(img_integral, upper_left, tmp_sum2);
    for (int i = 0; i < 3; ++i) {
        tmp_sum1[i] -= tmp_sum2[i];
    }
    
    // compute mean
    for (int i = 0; i < 3; ++i) {
        m_mean[i] = (1.0 * tmp_sum1[i]) / m_area;
    }
    
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
    tmp.x = m_roi.x + m_roi.width; tmp.y = m_roi.y + m_blocksize / 2; tmp.width = m_blocksize / 2; tmp.height = m_blocksize;
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
    tmp.x = m_roi.x + m_blocksize / 2; tmp.y = m_roi.y + m_roi.height; tmp.width = m_blocksize; tmp.height = m_blocksize / 2;
    computeMeanInnerRect(img_integral, tmp, m_compare);
    
    color_dist = dist(m_compare, m_mean);
    
    if (color_dist < threshold) {
        return false;
    }
    
    // little cube in the middle (left)
    tmp.x = m_roi.x; tmp.y = m_roi.y + m_blocksize; tmp.width = m_blocksize / 2; tmp.height = m_blocksize / 2;
    computeMeanInnerRect(img_integral, tmp, m_compare);
    
    color_dist = dist(m_compare, m_mean);
    
    if (color_dist < threshold) {
        return false;
    }
    
    // little cube in the middle (right)
    tmp.x = m_roi.x + m_blocksize; tmp.y = m_roi.y; tmp.width = m_blocksize / 2; tmp.height = m_blocksize / 2;
    computeMeanInnerRect(img_integral, tmp, m_compare);
    
    color_dist = dist(m_compare, m_mean);
    
    if (color_dist < threshold) {
        return false;
    }
    
    return true;
}

// ------------------------- BOTTLE VERY CLOSE 45P ------------------------------------

bool BottleVeryClose45P::match(const cv::Mat &img_integral, cv::Point p, double threshold) {
    if ((p.x - m_blocksize / 2) < 0
        || (p.y - m_blocksize / 2) < 0
        || (p.y + m_roi.height + m_blocksize / 2) >= img_integral.rows
        || (p.x + m_roi.width + m_blocksize / 2) >= img_integral.cols) {
        
        return false;
    }
    
    m_roi.x = p.x;
    m_roi.y = p.y;
    
    int tmp_sum1[3];
    int tmp_sum2[3];
    
    Rect lower_left(m_roi.x, m_roi.y + m_blocksize / 2, m_blocksize, m_blocksize);
    Rect upper_right(m_roi.x + m_blocksize / 2, m_roi.y , (m_blocksize * 3) / 2, m_blocksize);
    
    // sum up two blocks
    computeSum(img_integral, lower_left, tmp_sum1);
    computeSum(img_integral, upper_right, tmp_sum2);
    
    for (int i = 0; i < 3; ++i) {
        tmp_sum1[i] += tmp_sum2[i];
    }
    
    // subtract their intersection
    lower_left.x = m_roi.x + m_blocksize / 2;
    lower_left.y = m_roi.y + m_blocksize / 2;
    lower_left.width = m_blocksize / 2;
    lower_left.height = m_blocksize / 2;
    
    computeSum(img_integral, lower_left, tmp_sum2);
    for (int i = 0; i < 3; ++i) {
        tmp_sum1[i] -= tmp_sum2[i];
    }
    
    // compute mean
    for (int i = 0; i < 3; ++i) {
        m_mean[i] = (1.0 * tmp_sum1[i]) / m_area;
    }
    
    Rect tmp;
    double color_dist;
    
    // left
    tmp.x = m_roi.x - m_blocksize/ 2; tmp.y = m_roi.y + m_blocksize / 2; tmp.width = m_blocksize / 2; tmp.height = m_blocksize;
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
    tmp.x = m_roi.x + m_blocksize / 2; tmp.y = m_roi.y - m_blocksize / 2; tmp.width = m_blocksize; tmp.height = m_blocksize / 2;
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
    
    // little cube in the middle (left)
    tmp.x = m_roi.x; tmp.y = m_roi.y; tmp.width = m_blocksize / 2; tmp.height = m_blocksize / 2;
    computeMeanInnerRect(img_integral, tmp, m_compare);
    
    color_dist = dist(m_compare, m_mean);
    
    if (color_dist < threshold) {
        return false;
    }
    
    // little cube in the middle (right)
    tmp.x = m_roi.x + m_blocksize; tmp.y = m_roi.y + m_blocksize; tmp.width = m_blocksize; tmp.height = m_blocksize / 2;
    computeMeanInnerRect(img_integral, tmp, m_compare);
    
    color_dist = dist(m_compare, m_mean);
    
    if (color_dist < threshold) {
        return false;
    }
    
    return true;
}

// ------------------- BOTTLE VERY CLOSE 45N -----------------------

bool BottleVeryClose45N::match(const cv::Mat &img_integral, cv::Point p, double threshold) {
    if ((p.x - m_blocksize / 2) < 0
        || (p.y - m_blocksize / 2) < 0
        || (p.y + m_roi.height + m_blocksize / 2) >= img_integral.rows
        || (p.x + m_roi.width + m_blocksize / 2) >= img_integral.cols) {
        
        return false;
    }
    
    m_roi.x = p.x;
    m_roi.y = p.y;
    
    int tmp_sum1[3];
    int tmp_sum2[3];
    
    Rect upper_left(m_roi.x, m_roi.y, (m_blocksize * 2) / 3, m_blocksize);
    Rect lower_right(m_roi.x + m_blocksize / 2, m_roi.y + m_blocksize / 2 , m_blocksize, m_blocksize);
    
    // sum up two blocks
    computeSum(img_integral, upper_left, tmp_sum1);
    computeSum(img_integral, lower_right, tmp_sum2);
    
    for (int i = 0; i < 3; ++i) {
        tmp_sum1[i] += tmp_sum2[i];
    }
    
    // subtract their intersection
    upper_left.x = m_roi.x + m_blocksize;
    upper_left.y = m_roi.y + m_blocksize;
    upper_left.width = m_blocksize / 2;
    upper_left.height = m_blocksize / 2;
    
    computeSum(img_integral, upper_left, tmp_sum2);
    for (int i = 0; i < 3; ++i) {
        tmp_sum1[i] -= tmp_sum2[i];
    }
    
    // compute mean
    for (int i = 0; i < 3; ++i) {
        m_mean[i] = (1.0 * tmp_sum1[i]) / m_area;
    }
    
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
    tmp.x = m_roi.x + m_roi.width; tmp.y = m_roi.y + m_blocksize / 2; tmp.width = m_blocksize / 2; tmp.height = m_blocksize;
    computeMeanInnerRect(img_integral, tmp, m_compare);
    
    color_dist = dist(m_compare, m_mean);
    
    if (color_dist < threshold) {
        return false;
    }
    
    // top
    tmp.x = m_roi.x; tmp.y = m_roi.y - m_blocksize / 2; tmp.width = (m_blocksize * 3) / 2; tmp.height = m_blocksize / 2;
    computeMeanInnerRect(img_integral, tmp, m_compare);
    
    color_dist = dist(m_compare, m_mean);
    
    if (color_dist < threshold) {
        return false;
    }
    
    // bottom
    tmp.x = m_roi.x + m_blocksize; tmp.y = m_roi.y + m_roi.height; tmp.width = m_blocksize; tmp.height = m_blocksize / 2;
    computeMeanInnerRect(img_integral, tmp, m_compare);
    
    color_dist = dist(m_compare, m_mean);
    
    if (color_dist < threshold) {
        return false;
    }
    
    // little cube in the middle (left)
    tmp.x = m_roi.x; tmp.y = m_roi.y + m_blocksize; tmp.width = m_blocksize; tmp.height = m_blocksize / 2;
    computeMeanInnerRect(img_integral, tmp, m_compare);
    
    color_dist = dist(m_compare, m_mean);
    
    if (color_dist < threshold) {
        return false;
    }
    
    // little cube in the middle (right)
    tmp.x = m_roi.x + (m_blocksize * 3) / 2; tmp.y = m_roi.y; tmp.width = m_blocksize / 2; tmp.height = m_blocksize / 2;
    computeMeanInnerRect(img_integral, tmp, m_compare);
    
    color_dist = dist(m_compare, m_mean);
    
    if (color_dist < threshold) {
        return false;
    }
    
    return true;
}

// ----------------------------------- BEACON ------------------------------------------

bool Beacon::match(const cv::Mat &img_integral, cv::Point p, double threshold) {
    if  ((p.x - m_blocksize / 2) < 0
        || (p.y + m_roi.height + m_blocksize / 2) >= img_integral.rows
        || (p.x + m_roi.width + m_blocksize / 2) >= img_integral.cols) {
        
        return false;
    }
    
    m_roi.x = p.x;
    m_roi.y = p.y;
    m_roi.width = m_blocksize;
    m_roi.height = m_blocksize * 2;
    
    double mu_new[3] = {0.0, 0.0, 0.0};
    double mu_old[3] = {0.0, 0.0, 0.0};
    double color_dist;
    double color_dist_min = INFINITY;
    Rect r_tmp, r_max;
    
    
    // find maximal response in given row
    for (m_roi.x = p.x; m_roi.x < (img_integral.cols - m_roi.width - m_blocksize/2); m_roi.x += m_blocksize/2) {
        for (int i = 0; i < 4; ++i) {
            computeMeanInnerRect(img_integral, m_roi, mu_new);
            color_dist = dist(mu_new, m_color[i]);
            std::cout << color_dist << " ";
            if (color_dist < threshold) {
                if (color_dist < color_dist_min) {
                    color_dist_min = color_dist;
                    memcpy(mu_old, mu_new, 3 * sizeof(double));
                    r_max = m_roi;
                    m_corner = i > 0 ? i - 1 : i;
                }
            }
        }
        std::cout << std::endl;
    }
    
    if (!(color_dist_min < threshold)) {
        return false;
    }
    
    m_roi.x = r_max.x;
    m_roi.y = r_max.y;
    color_dist = dist(mu_new, m_color[m_corner]);
    
    // walk down the rows in the maximal column
    for (m_roi.y += m_blocksize/ 2; m_roi.y + m_roi.height < img_integral.rows; m_roi.y += m_blocksize/2) {
        computeMeanInnerRect(img_integral, m_roi, mu_new);
        
        if (color_dist > threshold) {
            break;
        }
    }
    
    m_roi.height =  m_roi.y;
    m_roi.y = p.y;
    
    return true;
    
}
