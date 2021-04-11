#ifndef MAZE_H
#define MAZE_H

#include <matplot/matplot.h>
#include "math.h"
#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <cmath>

#include <chrono>

using std::vector;
using namespace matplot;

//Global Variables
const float INIT_PROBABILITY = 0.5;
const float NO_INFO_PROBABILITY = 0.5;
const float OCCUPIED_PROBABILITY = 0.75;
const float LIKELY_EMPTY_PROBABILITY = 0.25;

struct LaserInfo
{
    float angleMax;
    float angleMin;
    float angleInc;
    std::vector<float> range_data;
};

struct Cell
{
    float inverseModel;
    float logit;
    float probability;
};

class Maze
{
private:
    uint16_t mazeWidth, mazeHeight;
    uint16_t refRow, refCol;
    uint16_t currRow, currCol;
    vector<vector<Cell>> data;
    float cellSize;
    float initLogit;
    figure_handle figure;
    axes_handle axes;
public:
    Maze(uint16_t maze_width, uint16_t maze_height, float cell_size, uint16_t start_row, uint16_t start_col, float pos_x, float pos_y);

    ~Maze();
    void init_maze();
    void init_figure();

    //gets the position of the of the object in the maze
    std::pair<uint16_t, uint16_t> getPos(float pos_x, float pos_y) const;

    void processLidarData(float yaw, const LaserInfo* laser_data, const float max_range);
    void draw();
    
    // main update function to update the maze
    void update(float pos_x, float pos_y, float yaw, const LaserInfo* laser_data, const float max_range);

    void updatePos(float pos_x, float pos_y);
    void updateOccupancyEstimate(std::vector<std::pair<int, int>>& points_of_interest, bool out_of_range, const uint16_t alpha, const float beta);

    // Calculates the latest inverse measurement model for a laser scanner and stores it in the cell
    void updateInverseMeasurement(std::vector<std::pair<int, int>>& points_of_interest, bool dest_out_of_range, const uint16_t alpha, const float beta);\
    void updateLogitMeasurement(std::vector<std::pair<int, int>>& points_of_interest);
    void updateProbability(std::vector<std::pair<int, int>>& points_of_interest);
    void update_image();

    // printing/logging functions
    void printProbabilities();
};
#endif