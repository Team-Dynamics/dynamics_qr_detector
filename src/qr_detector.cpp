/**
 * @file qr_detector.cpp
 * @author Stefan Dumberger
 * @brief QR Detector implementation
 * @version 1.0
 * @date 2020-09-30
 * 
 * @copyright Copyright (c) 2020
 * 
 */

// class definition
#include "dynamics_qr_detector/qr_detector.hpp"

// ROS dependencies
#include <cv_bridge/cv_bridge.h>
#include <dynamics_qr_msgs/QRCode.h>


QrDetector::QrDetector(ros::NodeHandle node, ros::NodeHandle priv_node)
 : _node(node), _priv_node(priv_node), _transport(_node), _scanner()
{
    // request ROS Parameters from parameter server
    getParams();

    // register publishers
    _pub_image = _transport.advertise("/qr_codes/image", 10);
    _pub_data = _node.advertise<dynamics_qr_msgs::QRCode>("/qr_codes/detected", 10);

    // config scanner library
    _scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);

    // register subscribers (allways at the end)
    _sub_rgb = _transport.subscribeCamera(_param_sub_topic, 10, &QrDetector::cb_imageRGB, this);
}

QrDetector::~QrDetector()
{
    // nothing to do
}

void QrDetector::getParams()
{
    // use string literals in this function only
    using namespace std::string_literals;

    // request paramerter from the parameter server
    // if not found, use a default value
    _param_sub_topic = _priv_node.param("sub_topic", "/camera/image"s);
}

void QrDetector::cb_imageRGB(const sensor_msgs::ImageConstPtr& rgb, const sensor_msgs::CameraInfoConstPtr& info_msg)
{
    cv_bridge::CvImagePtr cv_ptr;

    //transfer data into opencv
    try {
        cv_ptr = cv_bridge::toCvCopy(rgb, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("Error in cv_bridge: %s", e.what());
        return;
    }
    _frame = cv_ptr->image;

    // if QR Codes were found in the image, publish results
    if(0 < detectCodes())
    {
        publishImage(rgb->header);
        publishData(rgb->header);
    }
    
    //reset codes
    _codes.clear();
}

unsigned int QrDetector::detectCodes()
{
    // convert image to grayscale
    cv::Mat grayscale;
    cv::cvtColor(_frame, grayscale, CV_BGR2GRAY);

    // transfer image to zbar format
    zbar::Image image(
        grayscale.cols, 
        grayscale.rows, 
        "Y800", 
        grayscale.data, 
        grayscale.cols*grayscale.rows);

    // scan for QR Codes
    _scanner.scan(image);

    // process each code
    for (auto symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol)
    {
        QrCode qr;

        // save the content data
        qr.data = symbol->get_data();

        // save all corner coordinates
        for (auto i = 0; i < symbol->get_location_size(); i++)
        {
            qr.corners.push_back(
                cv::Point(
                    symbol->get_location_x(i), 
                    symbol->get_location_y(i)
                    )
                );
        }

        // collect all detected QR Codes
        _codes.push_back(qr);
    }

    return _codes.size();
}

void QrDetector::publishImage(const std_msgs::Header& header)
{
    // only process if someone is listening
    if(0 < _pub_image.getNumSubscribers())
    {
        //for each code
        for (auto &&c : _codes)
        {
            // draw a bounding box
            for (auto i = 1; i < c.corners.size(); ++i)
            {
                    cv::line(
                        _frame,                 // image
                        c.corners[i-1],         // start
                        c.corners[i],           // end
                        cv::Scalar(0, 0, 255),  // color
                        3                       // thickness
                        );
            }
            // close the box
            cv::line(
                _frame, 
                c.corners[0], 
                c.corners[c.corners.size()-1],
                cv::Scalar(0, 0, 255),          // OpenCV usese BGR format
                3
                );
        }

        // convert OpenCV image to ROS message
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(
            std_msgs::Header(header), 
            "bgr8", 
            _frame
            ).toImageMsg();
        
        // publish the image
        _pub_image.publish(msg);
    }
}

void QrDetector::publishData(const std_msgs::Header& header)
{
    // only process if someone is listening
    if (0 < _pub_data.getNumSubscribers())
    {
        // for each code
        for (auto &&c : _codes)
        {
            dynamics_qr_msgs::QRCode msg;

            // use image header
            msg.header = header;

            // copy data
            msg.data = c.data;

            // calculate center;
            for (auto &&i : c.corners)
            {
                msg.x += i.x;
                msg.y += i.y;
            }
            msg.x = msg.x / c.corners.size();
            msg.y = msg.y / c.corners.size();
            
            // publish data
            _pub_data.publish(msg);
        }
    }
}