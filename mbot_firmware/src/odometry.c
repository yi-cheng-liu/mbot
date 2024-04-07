#include "odometry.h"
#include <math.h>

int mbot_calculate_diff_body_vel(float wheel_left_vel, float wheel_right_vel, serial_twist2D_t *mbot_vel){
    mbot_vel->vx =  DIFF_WHEEL_RADIUS * (wheel_left_vel - wheel_right_vel) / 2.0f;
    mbot_vel->vy = 0;
    mbot_vel->wz =  DIFF_WHEEL_RADIUS * (-wheel_left_vel - wheel_right_vel) / (2.0f * DIFF_BASE_RADIUS);
    return 0; // Return 0 to indicate success
}
int mbot_calculate_diff_body_vel_imu(float wheel_left_vel, float wheel_right_vel, serial_mbot_imu_t imu, serial_twist2D_t *mbot_vel){
        //integrate IMU data to get V and w;  and we trust IMU so in the final result it counts 80%
        float sample_time=0.005;// see mbot.c line 187
        float imu_filtered[3]={0,0,0};
        // if(abs(imu.accel[0])>0.01)imu_filtered[0]=imu.accel[0];
        // if(abs(imu.accel[1])>0.01)imu_filtered[1]=imu.accel[1];
        // if(abs(imu.gyro[2])>0.01)imu_filtered[2]=imu.gyro[2];
        // imu_filtered[0]=imu.accel[0]-0.77;imu_filtered[1]=imu.accel[1]-0.37;imu_filtered[2]=imu.gyro[2];
        // mbot_vel->vx =  DIFF_WHEEL_RADIUS * (wheel_left_vel - wheel_right_vel) / 2.0f*0.2 + imu_filtered[0]*MAIN_LOOP_PERIOD*0.8;
        // mbot_vel->vy = 0+ imu_filtered[1]*MAIN_LOOP_PERIOD;
        //yaw shows rotate angular velocity which is imu.gyro[2]
        mbot_vel->wz =  DIFF_WHEEL_RADIUS * (-wheel_left_vel - wheel_right_vel) / (2.0f * DIFF_BASE_RADIUS)*0.8+ imu.gyro[2]*0.2;
        mbot_vel->vx =  DIFF_WHEEL_RADIUS * (wheel_left_vel - wheel_right_vel) / 2.0f;
        mbot_vel->vy = 0;
        //yaw shows rotate angular velocity which is imu.gyro[2]
        // mbot_vel->wz =  DIFF_WHEEL_RADIUS * (-wheel_left_vel - wheel_right_vel) / (2.0f * DIFF_BASE_RADIUS);
        return 0; // Return 0 to indicate success
}
int mbot_calculate_omni_body_vel(float wheel_left_vel, float wheel_right_vel, float wheel_back_vel, serial_twist2D_t *mbot_vel){
    mbot_vel->vx =  OMNI_WHEEL_RADIUS * (wheel_left_vel * INV_SQRT3 - wheel_right_vel * INV_SQRT3);
    mbot_vel->vy =  OMNI_WHEEL_RADIUS * (-wheel_left_vel / 3.0 - wheel_right_vel / 3.0 + wheel_back_vel * (2.0 / 3.0));
    mbot_vel->wz =  OMNI_WHEEL_RADIUS * -(wheel_left_vel + wheel_right_vel + wheel_back_vel) / (3.0f * OMNI_BASE_RADIUS);
    return 0; // Return 0 to indicate success
}
int mbot_calculate_omni_body_vel_imu(float wheel_left_vel, float wheel_right_vel, float wheel_back_vel, serial_mbot_imu_t imu, serial_twist2D_t *mbot_vel){
    return 0; // Return 0 to indicate success
}

int mbot_calculate_odometry(serial_twist2D_t mbot_vel, float dt, serial_mbot_imu_t imu, serial_pose2D_t *odometry){
    float vx_space = mbot_vel.vx * cos(odometry->theta) - mbot_vel.vy * sin(odometry->theta);
    float vy_space = mbot_vel.vx * sin(odometry->theta) + mbot_vel.vy * cos(odometry->theta);

    odometry->x += vx_space * dt*0.99;
    odometry->y += vy_space * dt*0.99;
    odometry->theta += mbot_vel.wz * dt;
    odometry->theta = (0.0*odometry->theta+1*imu.angles_rpy[2])*1.005;
    // Normalize theta to be between -pi and pi
    while (odometry->theta > M_PI) odometry->theta -= 2 * M_PI;
    while (odometry->theta <= -M_PI) odometry->theta += 2 * M_PI;

    return 0; // Return 0 to indicate success
}
