#include "map.h"
#include <chrono>
#include <matplot/matplot.h>

#define RAD2DEG(rad)((rad) * 180 / M_PI)
#define DEG2RAD(deg)((deg) * M_PI/180)

struct velocity{
    float linear;
    float angular;

    void set_vals(float _linear, float _angular){
        linear = _linear;
        angular = _angular;
    }
};

//Global Variables
const float MAX_RANGE = 3.5; // in meters
const uint16_t MAP_RENDER_CYCLE = 5;

float posX = 0.0, posY = 0.0, yaw = 0.0;
std::shared_ptr<LaserInfo> laserData = std::make_shared<LaserInfo>();

//Callbacks
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laserData->angleMin = msg->angle_min;
    laserData->angleMax = msg->angle_max;
    laserData->angleInc = msg->angle_increment;
    laserData->range_data = msg->ranges;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr & msg){
    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
    if (yaw < 0.0)
        yaw += 2 * M_PI;
    // ROS_INFO("Position: (%f,%f) Orientation:%f rad or %f degrees", posX, posY, yaw, RAD2DEG(yaw));
}

int main(int argc, char **argv)
{
    using namespace matplot;

    ros::init(argc, argv, "maze_explorer");
    ros::NodeHandle nh;
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    ros::Subscriber odom = nh.subscribe("odom", 1, &odomCallback);

    ros::Rate loop_rate(10);

    geometry_msgs::Twist vel;

    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    // TODO: Make the size of map dynamic as the car explores the environment
    std::unique_ptr<Map> map = std::make_unique<Map>(600, 600, 0.01, 300, 300, posX, posY);

    while(ros::ok()) {
        ros::spinOnce();

        map->update(posX, posY, yaw, laserData, MAX_RANGE);

        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();

        if (secondsElapsed > MAP_RENDER_CYCLE){
            map->update_image();
            start = std::chrono::system_clock::now();
        }

        loop_rate.sleep();
    }

    return 0;
}
