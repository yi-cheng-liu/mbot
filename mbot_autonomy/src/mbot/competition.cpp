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
    path.path.resize(48);

    mbot_lcm_msgs::pose2D_t nextPose;

    nextPose.x = 0.0;
    nextPose.y = 0.0f;
    nextPose.theta = 0.0f;
    path.path[0] = nextPose;

    nextPose.x = 0.3;
    nextPose.y = 0.0f;
    nextPose.theta = 0.0f;
    path.path[1] = nextPose;

    nextPose.x = 0.37;
    nextPose.y = -0.01;
    nextPose.theta = 0.0f;
    path.path[2] = nextPose;
    

    nextPose.x = 0.44;
    nextPose.y = 0.03;
    nextPose.theta = 0.0f;
    path.path[3] = nextPose;

    nextPose.x = 0.49;
    nextPose.y = 0.07;
    nextPose.theta = 0.0f;
    path.path[4] = nextPose;

    nextPose.x = 0.54;
    nextPose.y = 0.11f;
    nextPose.theta = 0.0f;
    path.path[5] = nextPose;

    nextPose.x = 0.58;
    nextPose.y = 0.17;
    nextPose.theta = 0.0f;
    path.path[6] = nextPose;
    
    nextPose.x = 0.6;
    nextPose.y = 0.24;
    nextPose.theta = 0.0f;
    path.path[7] = nextPose;

    nextPose.x = 0.61;
    nextPose.y = 0.3;
    nextPose.theta = 0.0f;
    path.path[8] = nextPose;

    nextPose.x = 0.62;
    nextPose.y = 0.37;
    nextPose.theta = 0.0f;
    path.path[9] = nextPose;

    nextPose.x = 0.64;
    nextPose.y = 0.44;
    nextPose.theta = 0.0f;
    path.path[10] = nextPose;

    nextPose.x = 0.68;
    nextPose.y = 0.49;
    nextPose.theta = 0.0f;
    path.path[11] = nextPose;

    nextPose.x = 0.72;
    nextPose.y = 0.54;
    nextPose.theta = 0.0f;
    path.path[12] = nextPose;

    nextPose.x = 0.78;
    nextPose.y = 0.58;
    nextPose.theta = 0.0f;
    path.path[13] = nextPose;

    nextPose.x = 0.85;
    nextPose.y = 0.6;
    nextPose.theta = 0.0f;
    path.path[14] = nextPose;

    nextPose.x = 0.91;
    nextPose.y = 0.61;
    nextPose.theta = 0.0f;
    path.path[15] = nextPose;

    nextPose.x = 0.98;
    nextPose.y = 0.6;
    nextPose.theta = 0.0f;
    path.path[16] = nextPose;

    nextPose.x = 1.05;
    nextPose.y = 0.58;
    nextPose.theta = 0.0f;
    path.path[17] = nextPose;

    nextPose.x = 1.1;
    nextPose.y = 0.54;
    nextPose.theta = 0.0f;
    path.path[18] = nextPose;

    nextPose.x = 1.15;
    nextPose.y = 0.49;
    nextPose.theta = 0.0f;
    path.path[19] = nextPose;

    nextPose.x = 1.19;
    nextPose.y = 0.44;
    nextPose.theta = 0.0f;
    path.path[20] = nextPose;

    nextPose.x = 1.21;
    nextPose.y = 0.37;
    nextPose.theta = 0.0f;
    path.path[21] = nextPose;

    nextPose.x = 1.22;
    nextPose.y = 0.3;
    nextPose.theta = 0.0f;
    path.path[22] = nextPose;

    nextPose.x = 1.23;
    nextPose.y = 0.24;
    nextPose.theta = 0.0f;
    path.path[23] = nextPose;

    nextPose.x = 1.25;
    nextPose.y = 0.17;
    nextPose.theta = 0.0f;
    path.path[24] = nextPose;

    nextPose.x = 1.28;
    nextPose.y = 0.11;
    nextPose.theta = 0.0f;
    path.path[25] = nextPose;

    nextPose.x = 1.33;
    nextPose.y = 0.07;
    nextPose.theta = 0.0f;
    path.path[26] = nextPose;

    nextPose.x = 1.39;
    nextPose.y = 0.03;
    nextPose.theta = 0.0f;
    path.path[27] = nextPose;

    nextPose.x = 1.46;
    nextPose.y = 0.01;
    nextPose.theta = 0.0f;
    path.path[28] = nextPose;

    nextPose.x = 1.52;
    nextPose.y = 0.0;
    nextPose.theta = 0.0f;
    path.path[29] = nextPose;

    nextPose.x = 1.83;
    nextPose.y = 0.0;
    nextPose.theta = 0.0f;
    path.path[30] = nextPose;

    nextPose.x = 1.83;
    nextPose.y = 0.91;
    nextPose.theta = 0.0f;
    path.path[31] = nextPose;

    nextPose.x = 1.82;
    nextPose.y = 0.98;
    nextPose.theta = 0.0f;
    path.path[32] = nextPose;

    nextPose.x = 1.80;
    nextPose.y = 1.05;
    nextPose.theta = 0.0f;
    path.path[33] = nextPose;

    nextPose.x = 1.76;
    nextPose.y = 1.1;
    nextPose.theta = 0.0f;
    path.path[34] = nextPose;

    nextPose.x = 1.71;
    nextPose.y = 1.15;
    nextPose.theta = 0.0f;
    path.path[35] = nextPose;

    nextPose.x = 1.66;
    nextPose.y = 1.19;
    nextPose.theta = 0.0f;
    path.path[36] = nextPose;

    nextPose.x = 1.59;
    nextPose.y = 1.21;
    nextPose.theta = 0.0f;
    path.path[37] = nextPose;

    nextPose.x = 1.52;
    nextPose.y = 1.22;
    nextPose.theta = 0.0f;
    path.path[38] = nextPose;

    nextPose.x = 0.3;
    nextPose.y = 1.22;
    nextPose.theta = 0.0f;
    path.path[39] = nextPose;

    nextPose.x = 0.24;
    nextPose.y = 1.21;
    nextPose.theta = 0.0f;
    path.path[40] = nextPose;

    nextPose.x = 0.17;
    nextPose.y = 1.19;
    nextPose.theta = 0.0f;
    path.path[41] = nextPose;

    nextPose.x = 0.11;
    nextPose.y = 1.15;
    nextPose.theta = 0.0f;
    path.path[42] = nextPose;

    nextPose.x = 0.07;
    nextPose.y = 1.1;
    nextPose.theta = 0.0f;
    path.path[43] = nextPose;

    nextPose.x = 0.03;
    nextPose.y = 1.05;
    nextPose.theta = 0.0f;
    path.path[44] = nextPose;

    nextPose.x = 0.01;
    nextPose.y = 0.98;
    nextPose.theta = 0.0f;
    path.path[45] = nextPose;

    nextPose.x = 0;
    nextPose.y = 0.91;
    nextPose.theta = 0.0f;
    path.path[46] = nextPose;

    nextPose.x = 0;
    nextPose.y = 0;
    nextPose.theta = 0.0f;
    path.path[47] = nextPose;


    path.path_length = path.path.size();

    lcm::LCM lcmInstance(MULTICAST_URL);
	std::cout << "publish to: " << CONTROLLER_PATH_CHANNEL << std::endl;
    lcmInstance.publish(CONTROLLER_PATH_CHANNEL, &path);
    sleep(1);

    return 0;
}
