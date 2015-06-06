#include "bgpattern.h"

int BGPattern::patterncount = 0;

// constructor with particular color
BGPattern::BGPattern(double g, double b, double r, int rows, int cols)
{
    m_mask = cv::Mat::zeros(rows, cols, CV_8UC1);
    m_curr[0] = g; m_curr[1] = b; m_curr[2] = r;
    m_acc[0] = m_acc[1] = m_acc[2] = 0.0;
    m_count = 0;
    m_id = patterncount;
    ++patterncount;
}

// constructor with particular color and th
BGPattern::BGPattern(double g, double b, double r, double th, int rows, int cols)
{
    m_mask = cv::Mat::zeros(rows, cols, CV_8UC1);
    m_mask = cv::Scalar(255);
//    cv::imshow("m", m_mask);
//    cv::waitKey();
    m_curr[0] = g; m_curr[1] = b; m_curr[2] = r;
    m_acc[0] = m_acc[1] = m_acc[2] = 0.0;
    m_count = 0;
    m_id = patterncount;
    m_dist_th = th;
    ++patterncount;
}

void BGPattern::setDistTh(double th)
{
    assert(th > 0.0);
    m_dist_th = th;
}

void BGPattern::reset()
{
    memset(m_acc, 0, sizeof(double) * 3);
    m_mask = cv::Scalar(0);
    m_count = 0;
}

bool BGPattern::addPixel(uchar *gbr, int idx)
{
    
    if( dist(gbr) > m_dist_th )
        return false;

    for(unsigned int i = 0; i < 3; ++i)
        m_acc[i] += gbr[i];
    ++m_count;
    
    uchar* maskptr = m_mask.data;
    maskptr[idx] = 1;
    return true;
}

bool BGPattern::addPixel(double *gbr, int idx)
{
    
    if( dist(gbr) > m_dist_th )
        return false;
    
    for(unsigned int i = 0; i < 3; ++i)
        m_acc[i] += gbr[i];
    ++m_count;
    
    uchar* maskptr = m_mask.data;
    maskptr[idx] = 1;
    return true;
}

double BGPattern::computeMean()
{
//    assert(m_count > 0);
    double old[3];
    getCurrMean(old);
    if (m_count > 1)
    {
        for(unsigned int i = 0; i < 3; ++i)
            m_curr[i] = m_acc[i] / m_count;
    }
    return dist(old);
}

void BGPattern::getCurrMean(double *mean)
{
    memcpy(mean, m_curr, sizeof(double) * 3);
}

double BGPattern::dist(uchar* a)
{
    double *b = m_curr;
    double dist = sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) + (a[2] - b[2]) * (a[2] - b[2]));
//    std::cout << dist << std::endl;
    return dist;
}

double BGPattern::dist(double* a)
{
    double *b = m_curr;
    double dist = sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) + (a[2] - b[2]) * (a[2] - b[2]));
    //    std::cout << dist << std::endl;
    return dist;
}

void BGPattern::maskImage(cv::Mat src, cv::Mat dst)
{
    // accept only char type matrices
    assert(src.depth() == CV_8U && dst.depth() == CV_8U);
    assert(src.channels() == 3 && dst.channels() == 1);
    assert(src.rows == dst.rows && src.cols == dst.cols);
    
    int rows = src.rows, cols = dst.cols;
    int npix = rows * cols;
    uchar *src_data = (uchar*)src.data;
    uchar *dst_data = (uchar*)dst.data;
    
//    for (unsigned int row = 0; row < rows; ++row)
//    {
//        uchar* src_row = src_data + 
//        for (unsigned int col = 0; col < col; ++col)
//    }
    
    double val0[3] = {m_curr[0], m_curr[1], m_curr[2]};
    // find new centroid
    reset();
    /*
    for (unsigned int i = 0; i < npix; ++i )
    {
//        std::cout << (int)src_data[0] << ", " << (int)src_data[1] << ", " << (int)src_data[2] << " " << addPixel(src_data)<< std::endl;
        
        addPixel(src_data);
        
        // move pointers
        src_data += 3;
    }
    computeMean();
    
    src_data = (uchar*)src.data; // reset pointer to beginning of image
    */
    
    for (unsigned int i = 0; i < npix; ++i ) // for next frame...
    {
        if (dist(src_data) < m_dist_th)
            dst_data[0] = 0; // 1 keeps background; 0 keeps objects
        else
            dst_data[0] = 1;
        
        // move pointers
        src_data += 3;
        dst_data += 1;
    }
    computeMean(); // for next frame...
//    std::cout << "center moved: " << dist(val0) << std::endl;
    
}

bool BGPattern::inRange(double dist) {
    return dist <= m_dist_th;
}

std::ostream& operator<< (std::ostream &out, BGPattern &p)
{
    out << "m_id = " << p.m_id << std::endl;
    out << "m_dist_th = " << p.m_dist_th << std::endl;
    out << "m_curr = [" << p.m_curr[0] << ", " << p.m_curr[1] << ", " << p.m_curr[2] << "]" << std::endl;
    out << "m_acc = [" << p.m_acc[0] << ", " << p.m_acc[1] << ", " << p.m_acc[2] << "]" << std::endl;
    out << "m_count = " << p.m_count;
    
    return out;
}

