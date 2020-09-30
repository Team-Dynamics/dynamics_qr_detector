/**
 * @file qr_detector.hpp
 * @author Stefan Dumberger
 * @brief QR Detector specification
 * @version 1.0
 * @date 2020-09-30
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#pragma once

// ROS dependencies
#include <ros/ros.h>
#include <image_transport/image_transport.h>

// OpenCV dependencies
#include <opencv2/opencv.hpp>

// QR Code detector library
#include <zbar.h>

/**
 * @brief QR Code detector ROS interface class
 * 
 */
class QrDetector
{
private:
    // ROS Handles
    ros::NodeHandle _node;
    ros::NodeHandle _priv_node;
    image_transport::ImageTransport _transport;

    // ROS Subscribers
    image_transport::CameraSubscriber _sub_rgb;

    // ROS Pubishers
    image_transport::Publisher _pub_image;
    ros::Publisher _pub_data;

    // ROS Parameters
    std::string _param_sub_topic;
    void getParams();

    // image detection members
    cv::Mat _frame;
    zbar::ImageScanner _scanner;
    
    struct QrCode
    {
        std::string data;
        std::vector<cv::Point> corners;
    };
    std::vector<QrCode> _codes;

    /**
     * @brief callback function for the camera subscriber
     * 
     * @param rgb image
     * @param info_msg camera info
     */
    void cb_imageRGB(
        const sensor_msgs::ImageConstPtr& rgb, 
        const sensor_msgs::CameraInfoConstPtr& info_msg
        );

    /**
     * @brief detect codes in the image
     * 
     * @return unsigned int number of detected codes
     */
    unsigned int detectCodes();

    /**
     * @brief publish a image with marked QR Codes
     * 
     * @param header header of the original image
     */
    void publishImage(
        const std_msgs::Header& header
        );

    /**
     * @brief publish the extracted data in the QR Codes
     * 
     * @param header header of the original image
     */
    void publishData(
        const std_msgs::Header& header
        );

public:
    /**
     * @brief Construct a new Qr Detector object
     * 
     * @param node public node handle
     * @param priv_node private node handle
     */
    QrDetector(
        ros::NodeHandle node, 
        ros::NodeHandle priv_node
        );

    /**
     * @brief Destroy the Qr Detector object
     * 
     */
    ~QrDetector();
};