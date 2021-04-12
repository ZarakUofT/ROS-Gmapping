#include "map.h"

Map::Map(uint16_t map_width, uint16_t map_height, float cell_size, uint16_t start_row, uint16_t start_col, float pos_x, float pos_y)
    :mapWidth(map_width + 2), mapHeight(map_height + 2), cellSize(cell_size),
        refRow(start_row), refCol(start_col)
{

    this->init_map();
    this->init_figure();
    this->updatePos(pos_x, pos_y);
}

Map::~Map()
{

}

void Map::init_map()
{
    this->data.resize(mapWidth, vector<Cell>(mapHeight));
    this->initLogit = Math::logit(INIT_PROBABILITY);

    for (int i = 0; i < mapWidth; i++){
        for (int j = 0; j < mapHeight; j++){
            this->data[i][j] = {INIT_PROBABILITY, this->initLogit, INIT_PROBABILITY};
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

void Map::update(float posX, float posY, float yaw, const std::shared_ptr<LaserInfo> laser_data, const float max_range)
{
    this->updatePos(posX, posY);
    this->updateMap(yaw, laser_data, max_range);
}

std::pair<uint16_t, uint16_t> Map::getPos(float pos_x, float pos_y) const
{
    uint16_t row, col;

    row = this->refRow + (uint16_t)(pos_x / this->cellSize);
    col = this->refCol + (uint16_t)(pos_y / this->cellSize);

    return std::make_pair(row, col);
}

void Map::updatePos(float pos_x, float pos_y)
{
    std::pair<uint16_t, uint16_t> pos = this->getPos(pos_x, pos_y);

    this->currRow = pos.first;
    this->currCol = pos.second;    
}

void Map::draw()
{
    this->figure->draw();
}

void Map::update_image()
{
    // Pass vectors of vector of floats as data
    std::vector<std::vector<float>> plot_data (this->mapWidth, std::vector<float>(this->mapHeight));

    for(int i = 0; i < this->mapWidth; i++){
        for(int j = 0; j < this->mapWidth; j++){
            plot_data[i][j] = this->data[i][j].probability;
        }
    }

    this->axes->image(plot_data, true);
    this->axes->color_box(true);
    this->axes->draw();
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
            this->data, this->currRow, this->currCol, yaw, phi, 
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

void Map::printProbabilities() {
    for (int i = 0; i < this->mapWidth; i++){
        for (int j = 0; j < this->mapHeight; j++){
            std::cout << this->data[i][j].probability << ", ";
        }
        std::cout << std::endl;  
    }    
}
