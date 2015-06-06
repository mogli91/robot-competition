/*
 * Template for code that runs on microcontrollers
 * TODO fill part in loop() and add own needed functions
 */
#include "MultiLinearCameraARD.h"

#include "prior.h"

unsigned char *image;
const unsigned int num_pixels = 612;

//prior position
enum POS{X, Y};
int pos[2]; //in cm

// position estimates
int pos_rough[2] = {0,0};
int pos_precise[2] = {0,0};

// pixel locations of beacons and respective deltas to correct compass
int peak_predict[4] = {0, 0, 0, 0};
int delta_px = 0;
int delta_angle = 0;

// direction variables
int direction_compass = 0;
int direction_idx = 0;
int image_y_coordinate = 0;

#define WORLDX 800
#define WORLDY 800

#define TILEX 50
#define TILEY 50

#define NUMX WORLDX/TILEX
#define NUMY WORLDY/TILEY

void setup()                    // run once, when the sketch starts
{

    Serial.begin(115200);
    
    //start pos 0 0
    pos[X] = 125; pos[Y] = 775; // 2 15
   
}

void loop()                       // run over and over again
{
    update_position_rough();
    //update_position_precise();
    Serial.print(pos[X]);
    Serial.print(" ");
    Serial.println(pos[Y]);
    Serial.println(pos_rough[X]);
    Serial.println(pos_rough[Y]);
//    Serial.print(" ");
//    Serial.println(iy);
  //Serial.println(delta_px);
    
    delay(1000);
}

void update_position_rough()
{
    int th = 0;
    
    ///ROUGH MODE
    // locate what square the robot is in, with a prior position.
    int x, y;
    int sums[9]; // sums of the intensities at the prior peak positions
    int index[4] = {0,0,0,0}; // index for the angles
    for(int i = 0; i < 9; i++) 
    {
        x = pos[X]/TILEX + i%3-1;
        y = pos[Y]/TILEY + i/3-1;
        if((x >= 0 && x < NUMX) && ( y >= 0 && y < NUMY ))
        {
            index[0] = 0; index[1] = 0; index[2] = 0; index[3] = 0;
            for(int j = 0; j < 4; j++)
            {
                if(pgm_read_word(&(locations2[x][y][j]))/*-pgm_read_word_near(angle2px + th)*/ < 0)
                  index[j] = pgm_read_word(&(locations2[x][y][j])) + 612;//locations2[x][y][j]/*-pgm_read_word_near(angle2px + th)*/ + 612;
                else
                  index[j] = pgm_read_word(&(locations2[x][y][j]));//locations2[x][y][j];//-pgm_read_word_near(angle2px + th);
                  
                  Serial.println(index[j]);
            }
            sums[i] = pgm_read_byte(&(simuIntensity[index[0]])) + pgm_read_byte(&(simuIntensity[index[1]])) + pgm_read_byte(&(simuIntensity[index[2]])) + pgm_read_byte(&(simuIntensity[index[3]])); 
            Serial.println(sums[i]);
        }
        else sums[i] = 0;
    }
    
    //reset x and y
    x = pos[X]/TILEX;
    y = pos[Y]/TILEY;
       
    //search for the sum maximum :: equivalent to the rough position
    int maxi = 4; // maximum i; begin at the robot usual position
    for(int i = 0; i < 9; i++) 
    {
        if(sums[i] > sums[maxi]) maxi = i;
    }
    
    Serial.println(maxi);
    
    //give the new rough position in function of the maximum i

    x += maxi%3-1;
    y += maxi/3-1;

    pos_rough[X] = x;
    pos_rough[Y] = y;
    
}

void update_position_precise()
{
    int th = 0;
    /// PRECISE MODE
    int p[2][2][2]; // four precalculated indexes around the robot position ex p00 x or y
    int w[2][2]; //sums of the 4 peaks
    int neighbor_peak_idx[4][4];
    int tmp = 0; //temp sum

    for(int i = 0; i < 4; i++)
    {
        p[i%2][i/2][X] = pos[X]/100 + i%2;
        p[i%2][i/2][Y] = pos[Y]/100 + i/2;
        
        for(int j = 0; j < NUM_BEACONS; j++) // 00 10 01 11
        {
            neighbor_peak_idx[i][j] = pgm_read_word(&(locations[ p[i%2][i/2][X] ][ p[i%2][i/2][Y] ][j])) + angle2px[th];
            tmp += image[neighbor_peak_idx[i][j]];
        }
        
        w[i%2][i/2] = tmp;
        tmp = 0;
    }
    
    pos_precise[X] = (w[0][0]*p[0][0][X] + w[1][0]*p[1][0][X] + w[1][1]*p[1][1][X] + w[0][1]*p[0][1][X]) / ( w[0][0] + w[1][0] + w[1][1] + w[0][1] );
    pos_precise[Y] = (w[0][0]*p[0][0][Y] + w[1][0]*p[1][0][Y] + w[1][1]*p[1][1][Y] + w[0][1]*p[0][1][Y]) / ( w[0][0] + w[1][0] + w[1][1] + w[0][1] );

    // reproject estimate back into the image (where would the peaks be given the new position)
    predict_peaks(neighbor_peak_idx[0], neighbor_peak_idx[1], neighbor_peak_idx[3], neighbor_peak_idx[2], w[0][0], w[1][0], w[1][1], w[0][1]);

    // compute reprojection error in image coordinates and angles
    delta_px = refine_peaks(peak_predict, 5);
    delta_angle = (delta_px * 612) / 360; // TODO check px to angle conversion
}

/*
 input:     index of neighbor peaks and respective weights
 computes:  pixel indexes of peaks in actual camera image
*/
void predict_peaks(int idx00[], int idx10[], int idx11[], int idx01[], int w00, int w10, int w11, int w01)
{
    int tmp[4];
    // predict where the 4 peaks should be in the actual camera image
    for (int i = 0; i < 4; ++i)
    {
        // peak by peak, compute weighted average among neighbors
        tmp[0] = idx00[i];
        tmp[1] = idx10[i];
        tmp[2] = idx11[i];
        tmp[3] = idx01[i];

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
        peak_predict[i] = (w00 * tmp[0] +w10 * tmp[1] +w11 * tmp[2] + w01 * tmp[3]) / (w00 + w10 + w11 + w01);
        // correct for offset introduced above
        if (peak_predict[i] >= 612)
        {
            peak_predict[i] -= 612;
        }
    }
}

/*
    input:    estimated peak indexes and a delta (> 0) to check for better response
    computes: the number of pixel (+ or -) that the given estimates need to be shifted 
*/
int refine_peaks(int peak_estimates[], int delta)
{
    int best_delta;
    int max_response = -1;
    int response = -1;
    int idx = 0;
    
    // go delta pixels left and delta pixels right
    for (int i = -delta; i <= delta; ++i)
    {
        response = 0;
        // sum up image intesities at tentative peak locations
        for (int j = 0; j < 4; ++j)
        {
            // adapt index in case of overflow or underflow
            idx = peak_estimates[j] + i;
            if (idx < 0)
                idx += 612;
            else if (idx >= 612)
                idx -= 612;

            response += image[idx];
        }

        if (response > max_response)
        {
            max_response = response;
            best_delta = delta;
        }
    }
    return best_delta;
}
