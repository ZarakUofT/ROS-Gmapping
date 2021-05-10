#ifndef MAP_H
#define MAP_H

#include <matplot/matplot.h>
#include "math.h"
#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <fstream>

#include <chrono>

using std::deque;
using namespace matplot;

// Global Variables
const float INIT_PROBABILITY = 0.5;
const float NO_INFO_PROBABILITY = 0.5;
const float OCCUPIED_PROBABILITY = 0.75;
const float LIKELY_EMPTY_PROBABILITY = 0.25;

const float RESIZE_THESHOLD = 3.5; // in Meters (Make sure lidar range is taken into account)

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

    Cell()
    : inverseModel(INIT_PROBABILITY), logit(0.f), probability(INIT_PROBABILITY){

    }

    Cell(float inverse_model, float log, float prob)
    : inverseModel(inverse_model), logit(log), probability(prob){

    }
};

class Map
{
private:
    uint16_t mapWidth, mapHeight;
    uint16_t refRow, refCol;
    uint16_t currRow, currCol;
    deque<deque<Cell>> data;
    float cellSize;
    float initLogit;
    figure_handle figure;
    axes_handle axes;

    // Held for plotting for graph on a different thread
    std::future<bool> futurePlotThread;
    bool threadCreatedForPlotting;
    bool stillPlotting;

    // Held saving probabiltiy data onto a file with a diff thread
    std::future<bool> futureFileSaveThread;
    bool threadCreatedForSaving;
    bool stillSaving;

    //Functions
    void init_map();
    void init_figure();

    void updateMap(float yaw, const std::shared_ptr<LaserInfo> laser_data, const float max_range);
    
    void updatePos(float pos_x, float pos_y);
    void updateOccupancyEstimate(std::vector<std::pair<int, int>>& points_of_interest, bool out_of_range, const uint16_t alpha, const float beta);
    
    // Calculates the latest inverse measurement model for a laser scanner and stores it in the cell
    void updateInverseMeasurement(std::vector<std::pair<int, int>>& points_of_interest, bool dest_out_of_range, const uint16_t alpha, const float beta);\
    void updateLogitMeasurement(std::vector<std::pair<int, int>>& points_of_interest);
    void updateProbability(std::vector<std::pair<int, int>>& points_of_interest);

    // Check if the map size needs to be increased
    //Increase the size and return true
    // return false if not resized
    bool resizedMap(int row, int col);

public:
    Map(uint16_t map_width, uint16_t map_height, float cell_size, uint16_t start_row, uint16_t start_col, float pos_x, float pos_y);

    ~Map();

    //gets the position of the of the object in the map
    std::pair<int, int> getPos(float pos_x, float pos_y) const;
    
    // main update function to update the map
    void update(float pos_x, float pos_y, float yaw, const std::shared_ptr<LaserInfo> laser_data, const float max_range);

    void update_image();

    // This function saves map data to a file
    void save_data(const std::string& fileName);

    // printing/logging functions
    void printProbabilities();

    template<typename T>
    void Debug(std::string label, std::vector<T> thingsToPrint = {});
};

// Prototypes for functions

// This function is called in a different thread to plot heatmap for probability of 
// occupancy of the map
bool display_image_thread_func(std::shared_ptr<std::vector<std::vector<float>>> plot_data, axes_handle& axes);
bool save_data_thread_func(std::shared_ptr<std::vector<std::vector<int>>> plot_data, const std::string& fileName);

#endif