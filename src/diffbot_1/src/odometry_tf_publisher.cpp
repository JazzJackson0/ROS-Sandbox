#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
float x;
float y;
float th;
float vx;
float vy;
float vth;


int main(int argc, char** argv) {

    ros::init(argc, argv, "odometry_tf_publisher");
    ros::NodeHandle = nh;
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);


    ros::Time current_time = ros::Time::now();
    ros::Time prev_time = ros::Time::now();
    tf::TransformBroadcaster tf_broadcaster;
    ros::Rate loop_rate(20);

    const double degree_convert = M_PI / 180;

    geometry_msgs::TransformStamped odom_transform;
    odom_transform.header.frame_id = "odom";
    odom_transform.child_frame_id = "base_footprint";

    while(ros::ok()) {
        
        // Get/Calculate Velocity Data from Encoders
        

        ros::Time current_time = ros::Time::now();
        float dt = (current_time - last_time).toSec();
        float delta_x = (vx * cos(th) - vy * sin(th)) * dt;
        float delta_y = (vx * sin(th) + vy * cos(th)) * dt;
        float delta_th = vth * dt;
        x += delta_x;
        y += delta_y;
        th = delta_th;

        geometry_msg::Quaternion odom_quaternion;
        odom_quaternion = tf::createQuaternionMsgFromRollPitchYaw(0, 0, th);

        // Poulate Odometry Transform Message
        odom_transform.header.stamp = current_time;
        odom_transform.transform.translation.x = x;
        odom_transform.transform.translation.y = y;
        odom_transform.transform.translation.z = 0;
        odom_transform.transform.rotation = tf::createQuaternionMsgFromYaw(th);

        // Populate Odometry Message
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.0.0;
        odom.pose.pose.orientation = odom_quaternion;
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.linear.z = 0.0;
        odom.twist.twist.angular.x = 0.0;
        odom.twist.twist.angular.y = 0.0;
        odom.twist.twist.angular.z = vth;
        prev_time = current_time;

        tf_broadcaster.sendTransform(odom_transform);
        odom_pub = publish(odom);
        loop_rate.sleep();

    }

    return 0;
}