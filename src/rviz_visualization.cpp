#include <ros/ros.h>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <visualization_msgs/Marker.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_visualizer");
    ros::NodeHandle nh;
    std::string path_adr;
    nh.getParam("/path_visualizer/path_adr", path_adr = "noFile");
    std::cout << "Read files from: " << std::endl;
    std::cout << path_adr << std::endl;
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    ros::Rate rate(30);

    float f = 0.0;
    while (ros::ok())
    {
        visualization_msgs::Marker points, line_strip, line_list;
        points.header.frame_id = line_strip.header.frame_id = "/world";
        points.header.stamp = line_strip.header.stamp = ros::Time::now();
        points.ns = line_strip.ns = "path_visualizer";
        points.action = line_strip.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

        points.id = 0;
        line_strip.id = 1;

        points.type = visualization_msgs::Marker::POINTS;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;

        // POINTS markers use x and y scale for width/height respectively
        points.scale.x = 0.05;
        points.scale.y = 0.05;

        // LINE_STRIP markers use only the x component of scale, for the line width
        line_strip.scale.x = 0.1;

        // Points are green
        points.color.g = 1.0f;
        points.color.a = 1.0;

        // Line strip is blue
        line_strip.color.b = 1.0;
        line_strip.color.a = 1.0;

        // Read path data from txt file
        std::ifstream path_file;
        float path[11];
        path_file.open(path_adr.c_str());
        while (!path_file.eof())
        {
            if (path_file >> path[0] >> path[1] >> path[2] >> //xyz position
                path[3] >> path[4] >> path[5] >>              //xyz velocity
                path[6] >> path[7] >> path[8] >>              //xyz acceleration
                path[9] >> path[10])                          //heading angular_vel
            {
                geometry_msgs::Point p;
                p.x = path[0];
                p.y = path[1];
                p.z = path[2];

                points.points.push_back(p);
                line_strip.points.push_back(p);
            }
        }

        marker_pub.publish(points);
        marker_pub.publish(line_strip);
        marker_pub.publish(line_list);

        rate.sleep();

        f += 0.04;
    }
}