#include <iostream>
#include <fstream>
#include <vector>
//From matplotlibcpp
#include "matplotlibcpp.h"

int main()
{
    std::ifstream in_file;
    in_file.open("io/path.txt");

    std::vector<double> x_pose_list, y_pose_list, z_pose_list,
        x_vel_list, y_vel_list, z_vel_list,
        x_acc_list, y_acc_list, z_acc_list,
        heading_list, ang_vel_list;
    double x_pose, y_pose, z_pose,
        x_vel, y_vel, z_vel,
        x_acc, y_acc, z_acc,
        heading, ang_vel;

    // Read and push read line into individual list for processing later
    if (!in_file)
    {
        std::cerr << "Unable to open file" << std::endl;
        exit(1);
    }
    while (!in_file.eof())
    {
        in_file >> x_pose >> y_pose >> z_pose >>
            x_vel >> y_vel >> z_vel >>
            x_acc >> y_acc >> z_acc >>
            heading >> ang_vel;

        x_pose_list.push_back(x_pose);
        y_pose_list.push_back(y_pose);
        z_pose_list.push_back(z_pose);
        x_vel_list.push_back(x_vel);
        y_vel_list.push_back(y_vel);
        z_vel_list.push_back(z_vel);
        x_acc_list.push_back(x_acc);
        y_acc_list.push_back(y_acc);
        z_acc_list.push_back(z_acc);
        heading_list.push_back(heading);
        ang_vel_list.push_back(ang_vel);
    }
    in_file.close();

    // Create the X-Axis
    std::vector<double>
        x_axis;
    for (int i = 1; i < x_pose_list.size() + 1; i++)
    {
        x_axis.push_back(i);
    }

    // Plotting of graph
    matplotlibcpp::backend("WXAgg");
    matplotlibcpp::clf();
    matplotlibcpp::title("Path Text File Plot");
    matplotlibcpp::named_plot("X_pose", x_axis, x_pose_list);
    matplotlibcpp::named_plot("Y_pose", x_axis, y_pose_list);
    matplotlibcpp::named_plot("Z_pose", x_axis, z_pose_list);
    matplotlibcpp::named_plot("X_vel", x_axis, x_vel_list);
    matplotlibcpp::named_plot("Y_vel", x_axis, y_vel_list);
    matplotlibcpp::named_plot("Z_vel", x_axis, z_vel_list);
    matplotlibcpp::named_plot("X_acc", x_axis, x_acc_list);
    matplotlibcpp::named_plot("Y_acc", x_axis, y_acc_list);
    matplotlibcpp::named_plot("Z_acc", x_axis, z_acc_list);
    matplotlibcpp::xlabel("Time [s]");
    matplotlibcpp::ylabel("Position [m], Velocity [m/s], Acceleration [m/s/s]");
    matplotlibcpp::legend();
    matplotlibcpp::show();
    matplotlibcpp::close();

    return 0;
}