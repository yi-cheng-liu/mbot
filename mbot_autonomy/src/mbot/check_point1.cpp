#include <utils/lcm_config.h>
#include <utils/lcm_config.h>
#include <mbot/mbot_channels.h>
#include <mbot_lcm_msgs/path2D_t.hpp>
#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <unistd.h>
#include <cmath>

int main(int argc, char** argv)
{

    std::cout << "Commanding robot to drive checkpoint1 maze\n";

    mbot_lcm_msgs::path2D_t path;
    path.path.resize(10);

    mbot_lcm_msgs::pose2D_t nextPose;

    nextPose.x = 0.0;
    nextPose.y = 0.0f;
    nextPose.theta = 0.0f;
    path.path[0] = nextPose;

    nextPose.x = 0.61;
    nextPose.y = 0.0f;
    nextPose.theta = 0.0f;
    path.path[1] = nextPose;

    nextPose.x = 0.61;
    nextPose.y = -0.61f;
    nextPose.theta = 0.0f;
    path.path[2] = nextPose;
    

    nextPose.x = 0.61*2;
    nextPose.y = -0.61;
    nextPose.theta = 0.0f;
    path.path[3] = nextPose;

    nextPose.x = 0.61*2;
    nextPose.y = 0.61;
    nextPose.theta = 0.0f;
    path.path[4] = nextPose;

    nextPose.x = 0.61*3;
    nextPose.y = 0.61f;
    nextPose.theta = 0.0f;
    path.path[5] = nextPose;

    nextPose.x = 0.61*3;
    nextPose.y = -0.61;
    nextPose.theta = 0.0f;
    path.path[6] = nextPose;
    
    nextPose.x = 0.61*4;
    nextPose.y = -0.61;
    nextPose.theta = 0.0f;
    path.path[7] = nextPose;

    nextPose.x = 0.61*4;
    nextPose.y = 0;
    nextPose.theta = 0.0f;
    path.path[8] = nextPose;

    nextPose.x = 0.61*5;
    nextPose.y = 0;
    nextPose.theta = 0.0f;
    path.path[9] = nextPose;

    path.path_length = path.path.size();

    lcm::LCM lcmInstance(MULTICAST_URL);
	std::cout << "publish to: " << CONTROLLER_PATH_CHANNEL << std::endl;
    lcmInstance.publish(CONTROLLER_PATH_CHANNEL, &path);
    sleep(1);

    return 0;
}
