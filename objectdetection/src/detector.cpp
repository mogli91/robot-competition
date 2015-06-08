#include "detector.h"

//Detector::Detector(Mat &frame, int camnum, float exposure, int f_height, int f_width)
Detector::Detector(int camnum, float exposure, int f_height, int f_width)
{
    this->f_height = f_height;
	this->f_width = f_width;
	this->exposure = exposure;
    this->frame = Mat::zeros(f_height, f_width, CV_8SC3);
    
    mask = Mat::zeros(f_height, f_width, CV_8UC1);
    
    double color_dist_th = 15;
    int blocksize = f_width / VISION_NUM_RAYS;
    int offset = 1.5 * blocksize;
    m_rangeFinder = new RangeFinder(f_height, f_width, blocksize, color_dist_th, offset);

    
    /*
     
    // -------------- Background segmentation using k-means --------------
    k_blur = Size(3,3);
    
    // morphology
    cross3 = getStructuringElement(MORPH_CROSS, Size(3,3));
    
    normalize = false;
    // initialize background patterns
    if (normalize) {
        double dist_th = 0.03;
        bg_tile = new BGPattern(n_color_tiles[0], n_color_tiles[1], n_color_tiles[2], dist_th, f_height, f_width);
        bg_grass = new BGPattern(n_color_grass[0], n_color_grass[1], n_color_grass[2], dist_th, f_height, f_width);
        bg_wood = new BGPattern(n_color_wood[0], n_color_wood[1], n_color_wood[2], dist_th, f_height, f_width);
        bg_obstacle = new BGPattern(n_color_obstacle[0], n_color_obstacle[1], n_color_obstacle[2], dist_th, f_height, f_width);
    } else {
        double dist_th = 30;
        bg_tile = new BGPattern(color_tiles[0], color_tiles[1], color_tiles[2], dist_th, f_height, f_width);
        bg_grass = new BGPattern(color_grass[0], color_grass[1], color_grass[2], dist_th, f_height, f_width);
        bg_wood = new BGPattern(color_wood[0], color_wood[1], color_wood[2], dist_th, f_height, f_width);
        bg_obstacle = new BGPattern(color_obstacle[0], color_obstacle[1], color_obstacle[2], dist_th, f_height, f_width);
    }
    
//    cout << *bg_tile << endl;
//    patterns.push_back(bg_tile);
//    patterns.push_back(bg_grass);
//    patterns.push_back(bg_wood);
//    patterns.push_back(bg_obstacle);
     
     */
    
    cap = VideoCapture(camnum); // open the default camera
    if (!cap.isOpened())  // check if we succeeded
    {
        cout << "no camera" << endl;
    }
    else {
        setSettings();
        printSettings();
    }
}

Detector::Detector(const string &filename)
{
    frame = cv::imread(filename);

    this->f_height = frame.rows;
    this->f_width = frame.cols;
    
    k_blur = Size(3,3);
    
    // morphology
    cross3 = getStructuringElement(MORPH_CROSS, Size(3,3));
    
    mask = Mat::zeros(f_height, f_width, CV_8UC1);
    
    double color_dist_th = 15;
    int blocksize = f_width / VISION_NUM_RAYS;
    int offset = 3 * blocksize;
    m_rangeFinder = new RangeFinder(f_height, f_width, blocksize, color_dist_th, offset);
    
}


Detector::~Detector(){
	for (int i = 0; i < patterns.size(); ++i) {
		delete(patterns[i]);
	}
    if (m_rangeFinder != NULL)
        delete(m_rangeFinder);
}

void Detector::detect()
{
    if (cap.isOpened()) {
        cap >> frame; // get a new frame from camera
    }
    
//    detect(frame);        // using k-means
    findRanges(frame);      // using region growing

}

void Detector::kMeansSegmentation(Mat img)
{
    clock_t t0_frame = clock();
    double elapsed_secs = 1.0;
    
    // 1. massive blur
    blur(img, img_blur, k_blur);

    // 2. threshold difference images
    maskImage(img_blur, mask);
    
    // 3. combine masks with logical AND (is done in 2 already)

    // 4. remove noise (opnening)
    morphologyEx(mask, mask, MORPH_OPEN, cross3);

    // 5. grow regions (dilation)
    morphologyEx(mask, mask, MORPH_DILATE, cross3);
    
    elapsed_secs = double(clock() - t0_frame) / CLOCKS_PER_SEC;
    cout << "\r" << 1.0 / elapsed_secs << " fps" << flush;
    
//        imshow("tile", patterns[0]->getMask() * 255);
//    imshow("tile", bg_tile->getMask() * 255);
//    imshow("grass", bg_grass->getMask() * 255);
//    imshow("wood", bg_wood->getMask() * 255);
//    imshow("obstacle", bg_obstacle->getMask() * 255);
    
}

void Detector::setSettings()
{

	if(cap.get(CV_CAP_PROP_FRAME_HEIGHT) != f_height)
			if(!cap.set(CV_CAP_PROP_FRAME_HEIGHT, f_height))
				cout << "Could not set frame height" << endl;
	if(cap.get(CV_CAP_PROP_FRAME_WIDTH) != f_width)
	    if(!cap.set(CV_CAP_PROP_FRAME_WIDTH, f_width))
	    	cout << "Could not set frame width" << endl;

//    cap.set(CV_CAP_PROP_GAIN, 0.3);
//    // http://stackoverflow.com/questions/15035420/configuring-camera-properties-in-new-ocv-2-4-3

	if(!cap.set(CV_CAP_PROP_EXPOSURE, exposure))
		cout << "Could not set exposure" << endl;

}

void Detector::printSettings()
{
	cout << "CV_CAP_PROP_FRAME_WIDTH " << cap.get(CV_CAP_PROP_FRAME_WIDTH) << endl;
	cout << "CV_CAP_PROP_FRAME_HEIGHT " << cap.get(CV_CAP_PROP_FRAME_HEIGHT) << endl;
	cout << "CV_CAP_PROP_BRIGHTNESS " << cap.get(CV_CAP_PROP_BRIGHTNESS) << endl;
	cout << "CV_CAP_PROP_SATURATION " << cap.get(CV_CAP_PROP_SATURATION) << endl;
	cout << "CV_CAP_PROP_CONTRAST " << cap.get(CV_CAP_PROP_CONTRAST) << endl;
	cout << "CV_CAP_PROP_HUE " << cap.get(CV_CAP_PROP_HUE) << endl;
	cout << "CV_CAP_PROP_GAIN " << cap.get(CV_CAP_PROP_GAIN) << endl;
	cout << "CV_CAP_PROP_EXPOSURE " << cap.get(CV_CAP_PROP_EXPOSURE) << endl;
}

Mat Detector::getFrame()
{
    return frame;
}

Mat Detector::getMask()
{
    return mask;
}

bool Detector::isReady()
{
	return cap.isOpened();
}

void Detector::maskImage(cv::Mat src, cv::Mat dst)
{
    // accept only char type matrices
    assert(src.depth() == CV_8U && dst.depth() == CV_8U);
    assert(src.channels() == 3 && dst.channels() == 1);
    assert(src.rows == dst.rows && src.cols == dst.cols);
    
    int rows = src.rows, cols = dst.cols;
    int npix = rows * cols;
    uchar *src_data = (uchar*)src.data;
    uchar *dst_data = (uchar*)dst.data;
    
    double px_normalized[3];
    
    BGPattern *pattern;
    double dist_min, dist_now;
    int assigned = -1;
    
    // get mask ptrs and prepare
    for (int p = 0; p < patterns.size(); ++p) {
        patterns[p]->reset();
    }
    
    for (unsigned int i = 0; i < npix; ++i ) // for next frame...
    {
        dist_min = 1000000;
        assigned = -1;
        
        // find closest pattern
        for (unsigned int p = 0; p < patterns.size(); ++p)
        {
            pattern = patterns[p];
            
            if (normalize) {
                normalize_px(src_data, px_normalized);
                dist_now = pattern->dist(px_normalized);
            } else {
                dist_now = pattern->dist(src_data);
            }
            
            if (pattern->inRange(dist_now) && (dist_now < dist_min)) {
                assigned = p;
            }
        }
        if (assigned != -1) {
            if (normalize) {
                normalize_px(src_data, px_normalized);
                patterns[assigned]->addPixel(px_normalized, i);
            } else {
                patterns[assigned]->addPixel(src_data, i);
            }
            dst_data[0] = 0; // 1 keeps background; 0 keeps objects
        } else {
            dst_data[0] = 1; // this might be something interesting
        }
        
        // move pointers
        src_data += 3;
        dst_data += 1;
    }
    
    // for next frame...
    for (int p = 0; p < patterns.size(); ++p) {
//        patterns[p]->computeMean();
        std::cout << p << " center moved: " << patterns[p]->computeMean() << std::endl;
    }
    
}

void Detector::normalize_px(uchar *src, double *dst)
{
    double ssum = 0;
    for(unsigned int s = 0; s < 3; ++s)
        ssum += (1.0 * src[s]) * src[s];
    ssum = sqrt(ssum);
    for(unsigned int s = 0; s < 3; ++s)
        dst[s] = src[s] / ssum;
}

void Detector::findRanges(cv::Mat img) {
//    clock_t t0_frame = clock();
//    double elapsed_secs = 1.0;
    
    frame = img;
    m_rangeFinder->rollOut(img, mask);
    
    computeMeasurement();
//    elapsed_secs = double(clock() - t0_frame) / CLOCKS_PER_SEC;
//    cout << "\r" << 1.0 / elapsed_secs << " fps" << flush;
    
//    imshow("ranges", mask * 255);
}

void Detector::computeMeasurement() {
    m_rangeFinder->getBeacon(m_measure.beacon);
    m_rangeFinder->getBottles(m_measure.bottles);
    m_rangeFinder->getRays(m_measure.rays);
//    m_rangeFinder->getBottleCoordinates(m_measure.bottles);
//    m_rangeFinder->getRayHeights(m_measure.rays);
//    m_rangeFinder->getBeacon(m_measure.beacon);
}

void Detector::getMeasurement(VisionMeasure &vm) {
//    vm.bottles.clear();
//    vm.bottles = m_measure.bottles;
//    vm.rays.clear();
//    vm.rays = m_measure.rays;
//    vm.beacon = m_measure.beacon;
    vm = m_measure;
}
