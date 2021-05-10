#include "map.h"

void Map::init_map()
{
    this->data.resize(mapWidth, deque<Cell>(mapHeight));
    this->initLogit = Math::logit(INIT_PROBABILITY);

    for (int i = 0; i < mapWidth; i++){
        for (int j = 0; j < mapHeight; j++){
            this->data[i][j] = Cell(INIT_PROBABILITY, this->initLogit, INIT_PROBABILITY);
        }
    }
}

void Map::init_figure() 
{
    this->figure = gcf();
    this->figure->ioff();
    this->axes = gca();
    this->axes->grid(false);
}

Map::Map(uint16_t map_width, uint16_t map_height, float cell_size, uint16_t start_row, uint16_t start_col, float pos_x, float pos_y)
    :mapWidth(map_width + 2), mapHeight(map_height + 2), cellSize(cell_size),
        refRow(start_row), refCol(start_col), stillPlotting(false), threadCreatedForPlotting(false),
        stillSaving(false), threadCreatedForSaving(false)
{

    this->init_map();
    this->init_figure();
    this->updatePos(pos_x, pos_y);
}

Map::~Map()
{

}

void Map::update(float posX, float posY, float yaw, const std::shared_ptr<LaserInfo> laser_data, const float max_range)
{
    this->updatePos(posX, posY);
    this->updateMap(yaw, laser_data, max_range);
}

std::pair<int, int> Map::getPos(float pos_x, float pos_y) const
{
    int row, col;

    row = this->refRow + (int)(pos_x / this->cellSize);
    col = this->refCol + (int)(pos_y / this->cellSize);

    return std::make_pair(row, col);
}

bool Map::resizedMap(int row, int col) {
    // Check if map needs resizing
    int resize_thresh_low = static_cast<int>((RESIZE_THESHOLD / this->cellSize));
    int resize_thresh_high = this->mapWidth + resize_thresh_low;

    if (row <= resize_thresh_low || col <= resize_thresh_low ||
        row >= resize_thresh_high || col >= resize_thresh_high) 
    {
        Math::resizeDeq(this->data, resize_thresh_low, resize_thresh_low);
        return true;
    }
    
    return false;
}

void Map::updatePos(float pos_x, float pos_y)
{
    std::pair<int, int> pos = this->getPos(pos_x, pos_y);

    if (this->resizedMap(pos.first, pos.second)){ 
        this->mapWidth = this->data.size();
        this->mapHeight = this->data[0].size();
        this->refRow = this->mapWidth / 2;
        this->refCol = this->mapHeight / 2;

        // find the correct pos again upon resize
        pos = this->getPos(pos_x, pos_y);
    }

    // should be a non-negative value by this point, so unsigned conversion should be fine
    this->currRow = (uint16_t) pos.first;
    this->currCol = (uint16_t) pos.second;    
}

void Map::updateMap(float yaw, const std::shared_ptr<LaserInfo> laser_data, const float max_range)
{
    if (!laser_data)
        return;

    std::pair<uint16_t, uint16_t> dest;
    std::vector<std::pair<int, int>> points;
    bool dest_out_of_range;

    // Alpha and Beta are used in the inverse measurement model
    // Think of a better way where to put/define these, proabably consts somewhere?
    const uint16_t alpha = 2;
    const float beta = laser_data->angleInc / 2;
    float phi = laser_data->angleMin;
    for (int i = 0; i < laser_data->range_data.size(); i++){
        dest = Math::pos_in_2d_array(
            this->mapWidth, this->mapHeight, this->currRow, this->currCol, yaw, phi, 
            laser_data->range_data[i] / this->cellSize, max_range / this->cellSize
        );
        
        dest_out_of_range = laser_data->range_data[i] > max_range;
        points = Math::bresenhamsAlgo(this->currRow, this->currCol, dest.first, dest.second);
        this->updateOccupancyEstimate(points, dest_out_of_range, alpha, beta);

        phi += laser_data->angleInc;
    }
}

void Map::updateOccupancyEstimate(std::vector<std::pair<int, int>>& points_of_interest, bool dest_out_of_range, 
                                    const uint16_t alpha, const float beta)
{
    this->updateInverseMeasurement(points_of_interest, dest_out_of_range, alpha, beta);
    this->updateLogitMeasurement(points_of_interest);
    this->updateProbability(points_of_interest);    
}

void Map::updateInverseMeasurement(std::vector<std::pair<int, int>>& points_of_interest, bool dest_out_of_range, 
                                    const uint16_t alpha, const float beta) 
{
    //Currently not using beta, but could be incorporated in the future
    for (int i = 0; i < points_of_interest.size(); i++){

        if (dest_out_of_range && i > points_of_interest.size() - alpha - 1){
            this->data[points_of_interest[i].first][points_of_interest[i].second].inverseModel = NO_INFO_PROBABILITY;
        }
        else if (i > points_of_interest.size() - alpha - 1){
            this->data[points_of_interest[i].first][points_of_interest[i].second].inverseModel = OCCUPIED_PROBABILITY;
        }
        else{
            this->data[points_of_interest[i].first][points_of_interest[i].second].inverseModel = LIKELY_EMPTY_PROBABILITY;
        }
    }
}

void Map::updateLogitMeasurement(std::vector<std::pair<int, int>>& points_of_interest) 
{
    for (int i = 0; i < points_of_interest.size(); i++){

        this->data[points_of_interest[i].first][points_of_interest[i].second].logit += \
            (Math::logit(this->data[points_of_interest[i].first][points_of_interest[i].second].inverseModel)  - this->initLogit);
    }    
}

void Map::updateProbability(std::vector<std::pair<int, int>>& points_of_interest)
{
    for (int i = 0; i < points_of_interest.size(); i++){

        this->data[points_of_interest[i].first][points_of_interest[i].second].probability = \
            Math::logit_inverse(this->data[points_of_interest[i].first][points_of_interest[i].second].logit);
    }
}

void Map::update_image()
{
    if (this->threadCreatedForPlotting)
        this->stillPlotting = this->futurePlotThread.wait_for(std::chrono::seconds(0))!=std::future_status::ready;
    if (this->stillPlotting)
        return;
    // Pass vectors of vector of floats as data
    std::vector<std::vector<float>> plot_data (this->mapWidth, std::vector<float>(this->mapHeight));

    for(int i = 0; i < this->mapWidth; i++){
        for(int j = 0; j < this->mapWidth; j++){
            if (this->data[i][j].probability < 0.3)
                plot_data[i][j] = 0.0;
            else if (this->data[i][j].probability > 0.75)
                plot_data[i][j] = 1.0;
            else
                plot_data[i][j] = 0.5;
        }
    }

    auto plot_data_ptr = std::make_shared<std::vector<std::vector<float>>>(plot_data);

    // Start a new thread
    this->futurePlotThread = std::async(std::launch::async, display_image_thread_func, plot_data_ptr, std::ref(this->axes));
    this->threadCreatedForPlotting = true;

    std::cout << "Continuing to main thread from Plotting thread" << std::endl;
}

void Map::save_data(const std::string& fileName) {


    if (this->threadCreatedForSaving)
        this->stillSaving = this->futureFileSaveThread.wait_for(std::chrono::seconds(0))!=std::future_status::ready;
    if (this->stillSaving)
        return;

    // Pass vectors of vector of floats as data
    std::vector<std::vector<int>> prob_data (this->mapWidth, std::vector<int>(this->mapHeight));

    for(int i = 0; i < this->mapWidth; i++){
        for(int j = 0; j < this->mapWidth; j++){
            if (this->data[i][j].probability < 0.3)
                prob_data[i][j] = 0;
            else if (this->data[i][j].probability > 0.75)
                prob_data[i][j] = 1;
            else
                prob_data[i][j] = -1; // unexplored
        }
    }

    auto save_data_ptr = std::make_shared<std::vector<std::vector<int>>>(prob_data);

    // Start a new thread
    this->futureFileSaveThread = std::async(std::launch::async, save_data_thread_func, save_data_ptr, std::ref(fileName));
    this->threadCreatedForPlotting = true;

    std::cout << "Continuing to main thread from Save Data Thread" << std::endl;
}

template<typename T>
void Map::Debug(std::string label, std::vector<T> thingsToPrint) {
    std::cout << label << ": ";

    for (auto &i : thingsToPrint){
        std::cout << i << ", ";
    }
    std::cout << std::endl;
}

void Map::printProbabilities() {
    for (int i = 0; i < this->mapWidth; i++){
        for (int j = 0; j < this->mapHeight; j++){
            std::cout << this->data[i][j].probability << ", ";
        }
        std::cout << std::endl;  
    }    
}

/* 
    Non-class Functions
*/
bool display_image_thread_func(std::shared_ptr<std::vector<std::vector<float>>> plot_data, axes_handle& axes) {
    if (!plot_data)
        return false;
    axes->image(*plot_data, true);
    axes->color_box(true);
    axes->draw();

    return true;
}

bool save_data_thread_func(std::shared_ptr<std::vector<std::vector<int>>> prob_data, const std::string& fileName) {
    std::ofstream outdata;
    auto& data = *prob_data;

    outdata.open(fileName, std::ios::trunc);

    if (!outdata){ // file couldn't be opened
        std::cerr << "Error: file could not be opened" << std::endl;
        return false;
    }

    for(int i = 0; i < data.size(); i++){
        for(int j = 0; j < data.size(); j++){
            outdata << data[i][j] << ",";
        }
        outdata << "\n";
    }
    return true;
}