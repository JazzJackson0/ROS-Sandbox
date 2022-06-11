#include <ros/ros.h>
#include <iostream>
float track_width = 0.0;
float v_linear = 0.0;
float v_rot = 0.0;
ros::Time = prev_time;
float right_encoder = 0.0;
float prev_right_encoder = 0.0;
float left_encoder = 0.0;
float prev_left_encoder = 0.0;
float travel_dist_left = 0.0;
float travel_dist_right = 0.0;
float ticks_per_meter = 0.0;
float x = 0.0;
float y = 0.0;
float th = 0.0;
geometry_msgs::Quaternion odom_quaternion;

void cmd_velCallback(const geometry_msgs::Twist &twist_aux) {

    geometry_msgs::Twist twist = twist_aux;
    float vel_x = twist_aux.linear.x;
    float vel_th = twist_aux.angular.z;
    float right_vel = 0.0;
    float left_vel = 0.0;

    if (vel_x == 0) {

        // Turning
        right_vel = vel_th * track_width / 2.0;
        left_vel = -1 * right_vel;
    }

    else if (vel_th == 0) {

        // Forward / Backward
        left_vel = right_vel = vel_x;
    }

    else {

        // Arc Movement
        left_vel = vel_x - vel_th * (track_width / 2.0);
        right_vel = vel_x + vel_th * (track_width / 2.0);

        v_linear = left_vel;
        v_rot = right_vel;
    }

    // Plug values into RIGHT & LEFT PIDs

}


int main(int argc char** argv) {

    ros::init(argc, argv, "base_controller");
    ros::NodeHandle nh;
    ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 10, cmd_velCallback);
    ros::Rate loop_rate(10);

    while(ros::ok()) {

        float dxy = 0.0;
        float dth = 0.0;
        ros::Time current_time = ros::Time::now();
        float dt;
        float vel_xy = dxy / dt;
        float vel_th = dth / dt;
        ros::spinOnce();
        dt = (current_time - prev_time).to_sec;
        prev_time = current_time;

        // Obtain Encoder Data

        if (right_encoder == 0) {

            travel_dist_left = 0.0;
            travel_dist_right = 0.0;
        }

        else {

            distance_left = (left_encoder - prev_left_encoder) / ticks_per_meter;
            distance_right = (right_encoder - prev_right_encoder) / ticks_per_meter;
        }

        prev_left_encoder = left_encoder;
        prev_right_encoder = right_encoder;
        dxy = (travel_dist_left + travel_dist_right) / 2.0;
        dth = (travel_dist_left - travel_dist_right) / track_width;

        if (dxy != 0) {

            x += dxy * cosf(dth);
            y += dxy * sinf(dth);
        }

        if (dth != 0) { th += dth; }

        odom_quaternion = tf::createQuaternionMsgFromRollPitchYaw(0, 0, th);
        loop_rate.sleep();
        

    }

    return 0;
}