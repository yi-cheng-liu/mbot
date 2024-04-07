#include <slam/action_model.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <utils/geometric/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>
#include <algorithm>


ActionModel::ActionModel(void)
: k1_(0.005f)
, k2_(0.025f)
, min_dist_(0.0025)
, min_theta_(0.02)
, initialized_(false)
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
    
    

}


void ActionModel::resetPrevious(const mbot_lcm_msgs::pose2D_t& odometry)
{
    previousPose_ = odometry;
}

void ActionModel::setCurrent(const mbot_lcm_msgs::pose2D_t& odometry)
{
    currentPose_ = odometry;
}


bool ActionModel::updateAction(const mbot_lcm_msgs::pose2D_t& odometry)
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////
    if((odometry.x-previousPose_.x)*(odometry.x-previousPose_.x)+(odometry.y-previousPose_.y)*(odometry.y-previousPose_.y)>=min_dist_*min_dist_||fabs(odometry.theta-previousPose_.theta)>=min_theta_)
    {
        setCurrent(odometry);
        return true;
    }
    return false;    // Placeholder
}

mbot_lcm_msgs::particle_t ActionModel::applyAction(const mbot_lcm_msgs::particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.
    mbot_lcm_msgs::particle_t newSample;
    
    
    double delta_s=sqrt((sample.pose.x-currentPose_.x)*(sample.pose.x-currentPose_.x)+(sample.pose.y-currentPose_.y)*(sample.pose.y-currentPose_.y));//length of travel line
    if(delta_s==0)delta_s=0.001;
    double theta_se=atan2(-sample.pose.y+currentPose_.y,-sample.pose.x+currentPose_.x);//theta of travel line
    double alpha=theta_se-sample.pose.theta;
    double delta_theta=currentPose_.theta-sample.pose.theta;
    std::random_device rd;
    std::mt19937 gen(rd());
    double k1=0.01,k2=0.01,k3=0.01;
    std::normal_distribution<float> d1(0,k1*fabs(alpha));
    std::normal_distribution<float> d2(0,k2*fabs(delta_s));
    std::normal_distribution<float> d3(0,k3*fabs(delta_theta-alpha));
    double e1=d1(gen),e2=d2(gen),e3=d3(gen);
    // double w1=(1.0/(sqrt(2.0 * M_PI)*k1*fabs(alpha)))*exp((-0.5*e1*e1)/(k1*fabs(alpha)*k1*fabs(alpha)));
    // double w2=(1.0/(sqrt(2.0 * M_PI)*k2*fabs(delta_s)))*exp((-0.5*e2*e2)/(k2*fabs(delta_s)*k2*fabs(delta_s)));
    // double w3=(1.0/(sqrt(2.0 * M_PI)*k3*fabs(delta_theta-alpha)))*exp((-0.5*e3*e3)/(k3*fabs(delta_theta-alpha)*k3*fabs(delta_theta-alpha)));
    double w1=exp((-0.5*e1*e1)/(k1*fabs(alpha)*k1*fabs(alpha)));
    double w2=exp((-0.5*e2*e2)/(k2*fabs(delta_s)*k2*fabs(delta_s)));
    double w3=exp((-0.5*e3*e3)/(k3*fabs(delta_theta-alpha)*k3*fabs(delta_theta-alpha)));
    newSample.parent_pose=sample.pose;
    newSample.pose.utime=currentPose_.utime;
    newSample.pose.x=sample.pose.x+(delta_s+e2)*cos(sample.pose.theta+alpha+e1);
    newSample.pose.y=sample.pose.y+(delta_s+e2)*sin(sample.pose.theta+alpha+e1);
    newSample.pose.theta=sample.pose.theta+delta_theta+e1+e3;
    newSample.weight=w1*w2*w3;
    // if(std::isnan(newSample.weight))std::cout<<"x:"<<k2<<"y:"<<fabs(delta_s)<<"t:"<<w3<<"\n";;
    // std::cout<<"weightsum of ProposalDis"<<w1<<"-----------"<<w2<<"________"<<w3<<"\n";
    return newSample;
}
