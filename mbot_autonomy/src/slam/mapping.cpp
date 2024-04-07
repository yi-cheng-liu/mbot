#include <slam/mapping.hpp>
#include <utils/grid_utils.hpp>
#include <numeric>
#include <chrono>
#include <list>
using namespace std::chrono;

Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
, initialized_(false)
{
}


void Mapping::updateMap(const mbot_lcm_msgs::lidar_t& scan,
                        const mbot_lcm_msgs::pose2D_t& pose,
                        OccupancyGrid& map)
{
    if (!initialized_)
        previousPose_ = pose;
    initialized_ = true;

    MovingLaserScan movingScan(scan, previousPose_, pose,2);

    /// TODO: Update the map's log odds using the movingScan  
    //
    // Hint: Consider both the cells the laser hit and the cells it passed through.

    for (const auto& ray : movingScan) {
        scoreEndpoint(ray, map);
        scoreRay(ray, map);
    }
    previousPose_ = pose;

    // for(int i=0;i<movingScan.size();i++)
    // {
    //     scoreEndpoint(movingScan.at(i),map);
    //     scoreRay(movingScan.at(i),map);
    // }
}

void Mapping::scoreEndpoint(const adjusted_ray_t& ray, OccupancyGrid& map)
{
    /// TODO: Implement how to score the cell that the laser endpoint hits  

    // Calculate the endpoint of the ray in world coordinates
    float odomX = ray.origin.x + ray.range * cos(ray.theta);
    float odomY = ray.origin.y + ray.range * sin(ray.theta);

    // Convert world coordinates to map coordinates
    int mapX = static_cast<int>((odomX - map.globalOrigin_.x) * map.cellsPerMeter_);
    int mapY = static_cast<int>((odomY - map.globalOrigin_.y) * map.cellsPerMeter_);

    // Calculate new log odds value for the cell
    CellOdds hit;
    if(map.logOdds(mapX,mapY)>=115) hit=125;
    else hit=map.logOdds(mapX,mapY)+10;
    // Update the map's log odds
    map.setLogOdds(mapX, mapY, hit);
}

void Mapping::scoreRay(const adjusted_ray_t& ray, OccupancyGrid& map)
{
    /// TODO: Implement how to score the cells that the laser ray passes through

    // Bresenham's line algorithm
    int startX = static_cast<int>((ray.origin.x - map.globalOrigin_.x) * map.cellsPerMeter_);
    int startY = static_cast<int>((ray.origin.y - map.globalOrigin_.y) * map.cellsPerMeter_);
    int endX = static_cast<int>((ray.origin.x + ray.range * cos(ray.theta) - map.globalOrigin_.x) * map.cellsPerMeter_);
    int endY = static_cast<int>((ray.origin.y + ray.range * sin(ray.theta) - map.globalOrigin_.y) * map.cellsPerMeter_);

    int dx = std::abs(endX - startX);
    int dy = std::abs(endY - startY);
    int err = dx - dy;
    int sx = (endX > startX) ? 1 : -1;
    int sy = (endY > startY) ? 1 : -1;

    int x = startX;
    int y = startY;

    CellOdds odd_value;
    if(map.logOdds(startX, startY)<=-124) odd_value=-127;
    else odd_value=map.logOdds(startX, startY)-3;
    map.setLogOdds(startX, startY, odd_value);

    while (x != endX || y != endY) {
        int e2 = err * 2;
        if (e2 >= -dy) {
            err -= dy;
            x += sx;
        }
        if (e2<=dx) {
            err += dx;
            y += sy;
        }
        if(map.logOdds(x, y)<=-124) odd_value=-127;
        else odd_value=map.logOdds(x, y)-3;
        map.setLogOdds(x, y, odd_value);
    }
    
}

/*
Takes the ray and map, and returns a vector of map cells to check
*/
std::vector<Point<int>> Mapping::bresenham(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    /// TODO: Implement the Bresenham's line algorithm to find cells touched by the ray.
    return {};
    
}

std::vector<Point<int>> Mapping::divideAndStepAlongRay(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    /// TODO: Implement an alternative approach to find cells touched by the ray. 
    return {};
    
}
