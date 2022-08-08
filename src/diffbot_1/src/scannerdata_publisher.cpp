#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <rpos/robot_platforms/slamware_core_platform.h>

using namespace rpos::robot_platforms;
using namespace std;


int main(int argc, char** argv) {

    
    ros::init(argc, argv, "scannerdata_publisher");
    ros::NodeHandle nh;
    ros::Publisher laser_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 50);

    SlamwareCorePlatform platform = SlamwareCorePlatform::connect("192.168.11.1", 1445);
    cout << "Base version: " << platform.getSDPVersion() << endl;
    
    double laser_freq = 40;
    int count = 0;
    ros::Rate r(1.0);
    unsigned int num_of_beams;
    double ranges[num_of_beams];
    double intensities[num_of_beams];

    while(nh.ok()) {

        // Get Laser Scan Data


        ros::Time scan_time = ros::Time::now();

        //Create Message
        sensor_msgs::LaserScan scan_i;
        scan_i.header.stamp = scan_time;
        scan_i.header.frame_id = "base_link";
        scan_i.angle_min = 0.0;
        scan_i.angle_max = 0.0;
        scan_i.angle_increment = 3.14 / num_of_beams;
        scan_i.time_increment = (1 / laser_freq) / num_of_beams;
        scan_i.range_min = 0.0;
        scan_i.range_max = 0.0;
        scan_i.ranges.resize(num_of_beams);  
        scan_i.intensities.resize(num_of_beams);

        for (int i = 0; i < num_of_beams; i++) {

            scan_i.ranges[i] = ranges[i];
            scan_i.intensities[i] = intensities[i];
        }

        laser_pub.publish(scan_i);
        ++count;
        r.sleep();
    }

    return 0;

}