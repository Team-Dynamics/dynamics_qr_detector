/**
 * @file node.cpp
 * @author Stefan Dumberger
 * @brief main node file for the ROS package
 * @version 1.0
 * @date 2020-09-30
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include <ros/ros.h>

#include "dynamics_qr_detector/qr_detector.hpp"

/**
 * @brief progrmm entry point
 * 
 * @param argc argument counter
 * @param argv argument vector
 * @return int 
 */
int main(int argc, char *argv[])
{
    // initialize ROS
    ros::init(argc, argv, "dynamics_qr_detector");

    // create NodeHandles for pubic and private namespaces
    ros::NodeHandle nh(""), nh_param("~");

    // QR Detector object
    QrDetector detector(nh, nh_param);

    // run until node is stopped
    ros::spin();

    // node doesn't generate runtime errors
    return 0;
}
