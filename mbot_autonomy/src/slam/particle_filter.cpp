#include <utils/grid_utils.hpp>
#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <mbot_lcm_msgs/pose2D_t.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <cassert>


ParticleFilter::ParticleFilter(int numParticles)
: kNumParticles_ (numParticles),
  samplingAugmentation_(0.5, 0.9, numParticles),
  distribution_quality(1),
  quality_reinvigoration_percentage(0.1)
{
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
}


void ParticleFilter::initializeFilterAtPose(const mbot_lcm_msgs::pose2D_t& pose)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<float> d1(0,0.005);
    std::normal_distribution<float> d2(0,0.005);
    std::normal_distribution<float> d3(0,0.01);
    for(int i=0;i<kNumParticles_;i++)
    {
        float e1=d1(gen),e2=d2(gen),e3=d3(gen);
        posterior_.at(i).pose.x=pose.x+e1;
        posterior_.at(i).pose.y=pose.y+e2;
        posterior_.at(i).pose.theta=pose.theta+e3;
        posterior_.at(i).pose.utime=pose.utime;
        posterior_.at(i).parent_pose=posterior_.at(i).pose;
        posterior_.at(i).weight=1/kNumParticles_;
    }

    

}

void ParticleFilter::initializeFilterRandomly(const OccupancyGrid& map)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
    RandomPoseSampler rps(map.bounds());
    for(int i=0;i<kNumParticles_;i++)
    {
        posterior_.at(i)=rps.get_particle(map);
        posterior_.at(i).weight=1/kNumParticles_;

    }

}

void ParticleFilter::resetOdometry(const mbot_lcm_msgs::pose2D_t& odometry)
{
    actionModel_.resetPrevious(odometry);
}


mbot_lcm_msgs::pose2D_t ParticleFilter::updateFilter(const mbot_lcm_msgs::pose2D_t& odometry,
                                                        const mbot_lcm_msgs::lidar_t& laser,
                                                        const OccupancyGrid& map)
{
    bool hasRobotMoved = actionModel_.updateAction(odometry);

    auto prior = resamplePosteriorDistribution(map);
    auto proposal = computeProposalDistribution(prior);
    resetOdometry(odometry);
    posterior_ = computeNormalizedPosterior(proposal, laser, map);
    /// TODO: Add reinvigoration step
    posteriorPose_ = estimatePosteriorPose(posterior_);

    posteriorPose_.utime = odometry.utime;

    return posteriorPose_;
}

mbot_lcm_msgs::pose2D_t ParticleFilter::updateFilterActionOnly(const mbot_lcm_msgs::pose2D_t& odometry)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);

    if(hasRobotMoved)
    {
        auto prior = resamplePosteriorDistribution();
        auto proposal = computeProposalDistribution(prior);
        resetOdometry(odometry);
        posterior_ = proposal;
    }

    posteriorPose_ = odometry;

    return posteriorPose_;
}


mbot_lcm_msgs::pose2D_t ParticleFilter::poseEstimate(void) const
{
    return posteriorPose_;
}


mbot_lcm_msgs::particles_t ParticleFilter::particles(void) const
{
    mbot_lcm_msgs::particles_t particles;
    particles.num_particles = posterior_.size();
    particles.particles = posterior_;
    return particles;
}


ParticleList ParticleFilter::resamplePosteriorDistribution(const OccupancyGrid& map,
                                                           const bool keep_best,
                                                           const bool reinvigorate)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////
    
    
    return importanceSample(posterior_.size(),posterior_);  // Placeholder
}


ParticleList ParticleFilter::resamplePosteriorDistribution(const bool keep_best,
                                                           const bool reinvigorate)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////
    
    return importanceSample(posterior_.size(),posterior_);
    // return lowVarianceSample(posterior_.size(),posterior_);  // Placeholder
}


void ParticleFilter::reinvigoratePriorDistribution(ParticleList& prior)
{
    // Augmentation: if sensor model suspects an average particle quality of
    //      less than 15%, invigorate
    if (distribution_quality < 0.15)  // TODO: make 0.15 a parameter
    {
        int count = 0;
        int max_count = floor(quality_reinvigoration_percentage * prior.size());

        std::random_device rd;
        std::default_random_engine generator(rd());
        auto ud01 = std::uniform_real_distribution<double>(0.0, 1.0);
        int step = std::max<int>(1, floor(ud01(generator) * prior.size() / max_count));

        for (int i = 0; i < max_count; i++)
        {
            prior[i*step] = randomPoseGen_.get_particle();
        }

    }

    // // Augmentation: randomize any unreasonable samples
    // if(map != nullptr)
    // {
    //     for (int i = 0; i < prior.size(); i++)
    //     {
    //         const auto& p = prior[i].pose;
    //         if(!map->isCellInGrid(p.x, p.y) ||
    //           (map->isCellInGrid(p.x, p.y) && (map->logOdds(p.x, p.y) >= 0)))
    //         {
    //             std::cout << "\tinvalid sample!!" << ", "
    //                 << !map->isCellInGrid(p.x, p.y) << ", "
    //                 << (map->isCellInGrid(p.x, p.y) && (map->logOdds(p.x, p.y) >= 0.0))
    //                 << std::endl;


    //             prior[i] = randomPoseGen_.get_particle();
    //         }
    //     }
    // }
}


ParticleList ParticleFilter::computeProposalDistribution(const ParticleList& prior)
{
    //////////// TODO: Implement your algorithm for creating the proposal distribution by sampling from the ActionModel
    ParticleList newEst;
    float weightSum=0;
    for(int i=0;i<prior.size();i++)
    {
        mbot_lcm_msgs::particle_t tmp;
        tmp=actionModel_.applyAction(prior.at(i));
        
        newEst.push_back(tmp);
        weightSum+=tmp.weight;
    }
    for(int i=0;i<newEst.size();i++)
    {
       newEst.at(i).weight/=weightSum;
    }
    return newEst;  // Placeholder
}


ParticleList ParticleFilter::computeNormalizedPosterior(const ParticleList& proposal,
                                                        const mbot_lcm_msgs::lidar_t& laser,
                                                        const OccupancyGrid& map)
{
    /////////// TODO: Implement your algorithm for computing the normalized posterior distribution using the
    ///////////       particles in the proposal distribution
    ParticleList newEst;
    double weightSum=0,tmp;int index=0;
    for(int i=0;i<proposal.size();i++)
    {
        tmp=sensorModel_.likelihood(proposal.at(i),laser,map);
        newEst.push_back(proposal.at(i));//proposal.weight would be nan
        newEst.at(index).weight*=tmp;
        if(!std::isnan(newEst.at(index).weight))
            weightSum+=newEst.at(index).weight;
        index+=1;
        
    }
    for(int i=0;i<newEst.size();i++)
    {
       newEst.at(i).weight/=weightSum;
    }
    return newEst;  // Placeholder
}


mbot_lcm_msgs::pose2D_t ParticleFilter::estimatePosteriorPose(const ParticleList& posterior)
{
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
    // Figure out which pose to take for the posterior pose
    // Weighted average is simple, but could be very bad
    // Maybe only take the best x% and then average.


    mbot_lcm_msgs::pose2D_t pose;
    ParticleList test=posterior,avgCon;
    mbot_lcm_msgs::particle_t tmp;
    for(int i=0;i<test.size()-1;i++)
    for(int j=i+1;j<test.size();j++)
    {
        if(test.at(j).weight>test.at(i).weight)
        {
            tmp=test.at(j);
            test.at(j)=test.at(i);
            test.at(i)=tmp;
        }
    }
    for(int i=0;i<test.size()/10;i++)
        avgCon.push_back(test.at(i));
    pose=computeParticlesAverage(avgCon);
    // std::cout<<"x:"<<pose.x<<"y:"<<pose.y<<"t:"<<pose.theta<<"sum:"<<avgCon.at(0).weight<<"\n";
    return pose;


}

mbot_lcm_msgs::pose2D_t ParticleFilter::computeParticlesAverage(const ParticleList& particles_to_average)
{
    mbot_lcm_msgs::pose2D_t avg_pose;
    avg_pose.x = 0.0;
    avg_pose.y = 0.0;
    avg_pose.theta = 0.0;
    double sum_weight = 0.0;

    // Aux variables to compute theta average
    double theta_x = 0.0;
    double theta_y = 0.0;
    for (auto &&p : particles_to_average)
    {
        avg_pose.x += p.weight * p.pose.x;
        avg_pose.y += p.weight * p.pose.y;
        theta_x += p.weight * std::cos(p.pose.theta);
        theta_y += p.weight * std::sin(p.pose.theta);

        sum_weight += p.weight;
    }
    avg_pose.x /= sum_weight;
    avg_pose.y /= sum_weight;
    theta_x /= sum_weight;
    theta_y /= sum_weight;
    avg_pose.theta = std::atan2(theta_y, theta_x);

    return avg_pose;
}
