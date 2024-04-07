#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <utils/grid_utils.hpp>
#include <utils/geometric/point.hpp>
SensorModel::SensorModel(void)
:   sigma_hit_(0.075),
	occupancy_threshold_(10),
	ray_stride_(7),
	max_ray_range_(1000),
    search_range(2),
    offset_quality_weight(3)
{
    initialize_bfs_offsets();
}

void SensorModel::initialize_bfs_offsets()
{
    /// TODO: Initialize the BFS offsets based on the search range 
    
}

double SensorModel::likelihood(const mbot_lcm_msgs::particle_t& sample, 
                               const mbot_lcm_msgs::lidar_t& scan, 
                               const OccupancyGrid& map)
{
    /// TODO: Compute the likelihood of the given particle using the provided laser scan and map. 
    double x=1;
    MovingLaserScan mls=MovingLaserScan(scan,sample.parent_pose,sample.pose,5);
    for(int i=0;i<mls.size();i++)
    {
        Point<float> preal=getRayEndPointOnMap(mls.at(i),map);
        Point<int> pmap=gridBFS(preal,map);
        for(int j=-1;j<=1;j++)
            for(int k=-1;k<=1;k++)
                if(map.logOdds(pmap.x+k,pmap.y+j)>=10){x+=1;break;}
        // else x-=1;

    }
    return x/mls.size(); // Placeholder
    
}

double SensorModel::scoreRay(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    /// TODO: Compute a score for a given ray based on its end point and the map. 
    // Consider the offset from the nearest occupied cell.  
    return 0.0; // Placeholder
    
}

double SensorModel::NormalPdf(const double& x)
{
    return (1.0/(sqrt(2.0 * M_PI)*sigma_hit_))*exp((-0.5*x*x)/(sigma_hit_*sigma_hit_));
}

Point<int> SensorModel::gridBFS(const Point<int> end_point, const OccupancyGrid& map)
{
    /// TODO: Use Breadth First Search to find the nearest occupied cell to the given end point. 
    int mapX=(end_point.x-map.globalOrigin_.x)*map.cellsPerMeter_;
    int mapY=(end_point.y-map.globalOrigin_.y)*map.cellsPerMeter_;
    return Point<int>(mapX,mapY); // Placeholder
    
}

Point<float> SensorModel::getRayEndPointOnMap(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    /// TODO: Calculate the end point of a given ray on the map 
    float odomX=ray.origin.x+ray.range*cos(ray.theta);
    float odomY=ray.origin.y+ray.range*sin(ray.theta);
    return Point<float>(odomX, odomY); // Placeholder
    
}
