#include "prior.h"

enum POS{X, Y};
enum NEIGHBORS{N00, N10, N11, N01};

#define CALIB_ANGLE_X_WRT_NORTH 334 // in degrees
#define CALIB_ANGLE_PX0_WRT_COMPASS 120
#define CALIB_PIXEL_PX0_WRT_COMPASS 204

struct Pose {
  int location[2];
  int angle_deg;
  int offset;    // something like angle in px
  int delta_old;
  float weights[4];
  int neighbors[4][2];
  int neighbor_peaks[4][NUM_BEACONS];
};

int rotate(int idx, int offset) {
   idx += offset;
   if (idx >= 612) 
     idx %= 612;
   else if (idx < 0)
     idx = 612 - (abs(idx) % 612);
   return idx;
}

void loadNeighborPeaks(Pose *p) {
  for (int j = 0; j < NUM_BEACONS; ++j)
  {
    p->neighbor_peaks[N00][j] = rotate(pgm_read_word(&(locations[p->neighbors[N00][X]][p->neighbors[N00][Y]][j])), p->offset);
    p->neighbor_peaks[N10][j] = rotate(pgm_read_word(&(locations[p->neighbors[N10][X]][p->neighbors[N10][Y]][j])), p->offset);
    p->neighbor_peaks[N11][j] = rotate(pgm_read_word(&(locations[p->neighbors[N11][X]][p->neighbors[N11][Y]][j])), p->offset);
    p->neighbor_peaks[N01][j] = rotate(pgm_read_word(&(locations[p->neighbors[N01][X]][p->neighbors[N01][Y]][j])), p->offset);    
  }
}

void computeWeights(Pose *p, unsigned char *img) {
  int total = 0;
  // integrate image responses
  for (int i = 0; i < 4; ++i)
  {
    int response = 0;
    for (int j = 0; j < NUM_BEACONS; ++j)
    {
//      int idx = rotate(p->neighbor_peaks[i][j], p->offset); // account for angular offset
      int idx = p->neighbor_peaks[i][j]; // offset is already removed at loading time
      response += img[idx];
    }
    p->weights[i] = response;
    total += response;
  }
  // normalize weights
  for (int i = 0; i < 4; ++i)
    p->weights[i] /= total;
}

void computeLocation(Pose *p)
{
  // assuming normalized weights
  float tmp_x = 0;
  float tmp_y = 0;  
  for (int i = 0; i < 4; ++i)
  {
    tmp_x += p->weights[i] * p->neighbors[i][X];
    tmp_y += p->weights[i] * p->neighbors[i][Y];
  }
  p->location[X] = (int)(tmp_x * TILEX);
  p->location[Y] = (int)(tmp_y * TILEY);  
}

int updateNeighbors(Pose *p)
{
  int x = p->location[X]; // in cm
  int y = p->location[Y];
  int shift_x = 0;
  int shift_y = 0;
  
  int current_border = p->neighbors[N00][X];
  // shift window in negative  x direction if needed AND possible
  if (current_border > 0 && (x - current_border * TILEX) < (TILEX/2))
    shift_x = -1;
  
  // shift window in positive x direction if needed AND possible  
  current_border = p->neighbors[N10][X];
  if (current_border < (NUM_X - 1) && (current_border * TILEX - x) < (TILEX/2))
    shift_x = 1;
  
  current_border = p->neighbors[N00][Y];
  // shift window in negative  y direction if needed AND possible
  if (current_border > 0 && (y - current_border * TILEY) < (TILEY/2))
    shift_y = -1;
  
  // shift window in positive x direction if needed AND possible  
  current_border = p->neighbors[N01][Y];
  if (current_border < (NUM_Y - 1) && (current_border * TILEY - y) < (TILEY/2))
    shift_y = 1;
    
  for (int i = 0; i < 4; ++i)
  {
    p->neighbors[i][X] += shift_x;
    p->neighbors[i][Y] += shift_y;
  }
  return (shift_x + 1)* 10 + shift_y + 1;
}

void predictPeaks(Pose *p, int peak_predict[])
{
    int tmp[4];
    // predict where the 4 peaks should be in the actual camera image
    for (int i = 0; i < 4; ++i)
    {
        // take i-th peak, compute weighted average among neighbors
        tmp[N00] = p->neighbor_peaks[N00][i];
        tmp[N10] = p->neighbor_peaks[N10][i];
        tmp[N11] = p->neighbor_peaks[N11][i];
        tmp[N01] = p->neighbor_peaks[N01][i];

        // find highest index
        int max_idx = tmp[0];
        for (int j = 1; j < 4; ++j)
        {
            if (tmp[j] > max_idx) 
            {
                max_idx = tmp[j];
            }
        }

        // correct indexes to ensure correct computation of 'mean' angle
        for (int j = 0; j < 4; ++j)
        {
            if ((max_idx - tmp[j]) > 306) // in px coordinates (not angles) 
            {
                tmp[j] += 612;
            }
        }
        
        float sum = 0;
        for (int j = 0; j < 4; ++j)
          sum += p->weights[j] * tmp[j];
         
        // correct for offset introduced above
        if (sum >= 612)
        {
            sum -= 612;
        }
        peak_predict[i] = (int)sum;
    }
}

/*
    input:    estimated peak indexes and a delta (> 0) to check for better response
    computes: the number of pixel (+ or -) that the given estimates need to be shifted 
*/
int refinePeaks(unsigned char *img, int peak_estimates[], int delta)
{
    delta = abs(delta);
    delta = delta % 612;
    if (delta < 5) delta = 5;
    
    int best_delta;
    int max_response = -1;
    long int response = -1;
    long int intensity = 0;
    int idx = 0;
    
    // go delta pixels left and delta pixels right
    for (int i = -delta; i <= delta; ++i)
    {
        response = 0;
        // sum up image intesities at tentative peak locations
        for (int j = 0; j < 4; ++j)
        {
            // adapt index in case of overflow or underflow
            idx = rotate(peak_estimates[j], i);
            intensity = img[idx];
            response += (intensity * intensity);
        }

        if (response > max_response)
        {
            max_response = response;
            best_delta = i;
        }
    }
//    Serial.print("max response: "); Serial.println(max_response);
    return best_delta;
}

int offset2Degree(int offset)
{
  return pgm_read_word(&(px2angle[rotate(0, - offset + CALIB_PIXEL_PX0_WRT_COMPASS)]));
}

void updateAngle(Pose *p, int angle) 
{
  if (p->angle_deg < 0)
    p->angle_deg = angle;
  else
  {
    float estimate = offset2Degree(p->offset);
    float max_, min_;
    if (estimate > angle)
    {
      max_ = estimate; min_ = angle;
    }
    else
    {
      max_ = angle; min_ = estimate;
    }
    
    if ((max_ - min_) > 180)
      min_ += 360;
    
    int result = (int)((min_ + max_) / 2);
    if (result >= 360)
      result -= 360;
    
    p->angle_deg = result;
  }
}

int transformAngleWorld2Local(int angle_deg_raw)
{
  angle_deg_raw -= CALIB_ANGLE_X_WRT_NORTH;
  if (angle_deg_raw < 0) angle_deg_raw += 360;
  if (angle_deg_raw >= 360) angle_deg_raw -= 360;
  return angle_deg_raw;
}

int degree2Offset(int angle_deg)
{
  return rotate(0, CALIB_PIXEL_PX0_WRT_COMPASS - pgm_read_word(&(angle2px[angle_deg])));
}
