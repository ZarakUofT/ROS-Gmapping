#ifndef MATH_H
#define MATH_H

#include <iostream>
#include <vector>
#include <math.h>
#include <limits>
#include <algorithm>

namespace Math
{

//Make sure to only pass in Numeric Types to this
//This does not support string literals
//For straight lines, this should return infinity
//We return double in order to handle the case of ints being passed
template<typename T>
double compute_slope(T x1, T y1, T x2, T y2)
{
    if (y2 - y1 == 0.)
        return 0.;

    //Check for division by zero
    if ((x2 - x1) == 0.){
        (y2 > y1) ? std::numeric_limits<double>::infinity(): -std::numeric_limits<double>::infinity();
    }else{
        return (y2 - y1) / (x2 - x1);
    }

    return 0.;  
}

void lowGradient(int x1, int y1, int x2, int y2, std::vector<std::pair<int, int>>& res)
{
    int dx = x2 - x1;
    int dy = y2 - y1;
    int yi = 1;
    
    if (dy < 0){
        yi = -1;
        dy = -dy;
    }

    if (dy == 0)
        yi = 0;

    int D = 2 * dy - dx;
    int y = y1;

    for(int i = x1; i <= x2;){
        res.push_back(std::make_pair(i, y));
        if (D > 0){
            y = y + yi;
            D = D + (2 * (dy - dx));
        }else{
            D = D + 2*dy;
        }

        if (x1 > x2)
            i--;
        else
            i++;
    }
}

void highGradient(int x1, int y1, int x2, int y2, std::vector<std::pair<int, int>>& res)
{
    int dx = x2 - x1;
    int dy = y2 - y1;
    int xi = 1;

    if (dx < 0){
        xi = -1;
        dx = -dx;
    }

    if (dx == 0)
        xi = 0;

    int D = 2 * dy - dx;
    int x = x1;

    for(int i = y1; i <= y2;){
        res.push_back(std::make_pair(x, i));
        if (D > 0){
            x = x + xi;
            D = D + (2 * (dx - dy));
        }else{
            D = D + 2*dx;
        }

        if (y1 > y2)
            i--;
        else
            i++;
    }
}

//This function takes in integer coordinates and returns a vector of pairs containing x and y coordinates of the line
//joining the two points
std::vector<std::pair<int, int>> bresenhamsAlgo(int x1, int y1, int x2, int y2) 
{
    std::vector<std::pair<int, int>> res;
    if (abs(y2 - y1) < abs(x2 - x1)){
        if (x1 > x2)
            lowGradient(x2, y2, x1, y1, res);
        else
            lowGradient(x1, y1, x2, y2, res);
    }else{
        if (y1 > y2)
            highGradient(x2, y2, x1, y1, res);
        else
            highGradient(x1, y1, x2, y2, res);
    }

    //Make sure it's returned in the right format
    if (res[0].first != x1 && res[0].second != x2)
        std::reverse(res.begin(), res.end());

    return res;
}

// Determine the destination pos in a 2D array given the orientation and range value
template<typename T>
std::pair<uint16_t, uint16_t> pos_in_2d_array(const std::vector<std::vector<T>>& m, 
    const uint16_t pos_x, const uint16_t pos_y, 
    const float yaw, const float phi, float range, const float r_max)
{
    if (!m.size())
        return {};

    int row = pos_x, col = pos_y;
    if (range > r_max){
        range = r_max;
    }
    
    row += round(range * (float)cos((double)(yaw + phi))); 
    col += round(range * (float)sin((double)(yaw + phi))); 

    if (row < 0)
        row = 0;
    else if (row > m.size() - 1)
        row = m.size() - 1;
    
    if (col < 0)
        col = 0;
    else if (col > m[0].size() - 1)
        col = m[0].size() - 1;
    
    return std::make_pair((uint16_t)row, (uint16_t)col);
}

//Converts probability into logit function
inline float logit(float p)
{
    return log(p / (1 - p));
}

//Converts logit back to probability
inline float logit_inverse(float l)
{
    return exp(l) / (1 + exp(l));
}

// returns a vector of n evenly spaced point 
std::vector<float> linspace(const float x1, const float x2, const int n)
{
    if (n <= 0)
        return std::vector<float>(0);
   
    std::vector<float> retval (n);

    float diff = (x2 - x1) / (n - 1);
    float val = x1;
    for (int i = 0; i < n; i++){
        retval[i] = val;
        val += diff;
    }
    return retval;
} 

}

#endif