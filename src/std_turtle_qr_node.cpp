#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include <zbar.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>

struct QrCode
{
  std::string data;
  std::vector<cv::Point> corners;
};

class QrDetector {
    ros::NodeHandle _node;
    image_transport::ImageTransport _transport;

    image_transport::Publisher _pubImage;
    ros::Publisher _pubData;
    image_transport::CameraSubscriber _subRGB;

    cv::Mat _frame;
    zbar::ImageScanner _scanner;

    std::vector<QrCode> _codes;

    void detectCodes()
    {
        cv::Mat greyscale;
        cv::cvtColor(_frame, greyscale, CV_BGR2GRAY);

        zbar::Image image(greyscale.cols, greyscale.rows, "Y800", greyscale.data, greyscale.cols*greyscale.rows);
        _scanner.scan(image);

        for (zbar::Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol)
        {
            QrCode q;

            q.data = symbol->get_data();
            for (unsigned int i = 0; i < symbol->get_location_size(); i++)
            {
                q.corners.push_back(cv::Point(symbol->get_location_x(i), symbol->get_location_y(i)));
            }

            _codes.push_back(q);
        }

        // clean up
        image.set_data(NULL, 0);
    }

    void publishCamera(const std_msgs::Header &header)
    {
        if(_pubImage.getNumSubscribers() > 0)
        {
            for (size_t i = 0; i < _codes.size(); i++) {
                for (unsigned int j = 0; j < 4; j++)
                {
                    cv::line(_frame, _codes[i].corners[j], _codes[i].corners[(j + 1) % 4], cv::Scalar(0, 0, 255), 3);
                }

                // add text
                const int text_thickness = 2;
                const char* text = _codes[i].data.c_str();
                int baseline = 0;
                cv::Size textsize = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 1, text_thickness, &baseline);
                cv::Point text_pos;
                text_pos.x = (_codes[i].corners[0].x + _codes[i].corners[1].x + _codes[i].corners[2].x + _codes[i].corners[3].x - 2*textsize.width)/4;
                text_pos.y = MAX(MAX(_codes[i].corners[0].y,_codes[i].corners[1].y), MAX(_codes[i].corners[2].y, _codes[i].corners[3].y)) + textsize.height;

                cv::putText(_frame, _codes[i].data, text_pos, cv::FONT_HERSHEY_SIMPLEX, 1, cvScalar(0,255,0), text_thickness, CV_AA);
            }

            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(header), "bgr8", _frame).toImageMsg();
            _pubImage.publish(msg);
        }
    }

    void publishCodes(const std_msgs::Header &header, const sensor_msgs::CameraInfoConstPtr& info_msg)
    {
        std_msgs::String msg;
        for (size_t i = 0; i < _codes.size(); i++) {
            msg.data = _codes[i].data;

            _pubData.publish(msg);
        }
    }

public:
    QrDetector()
    : _transport(_node)
    {
        _subRGB = _transport.subscribeCamera("/camera/image", 10, &QrDetector::cb_imageRGB, this);

        _pubImage = _transport.advertise("/qr_codes/image", 10);
        _pubData = _node.advertise<std_msgs::String>("/qr_codes/text", 10);

        _scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
    }

    ~QrDetector() { }

    void cb_imageRGB(const sensor_msgs::ImageConstPtr& rgb, const sensor_msgs::CameraInfoConstPtr& info_msg)
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

        detectCodes();
        publishCodes(rgb->header, info_msg);
        publishCamera(rgb->header);

        _codes.clear();
    }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtle_qr_detector");

  QrDetector detector;

  ros::spin();

  return 0;
}