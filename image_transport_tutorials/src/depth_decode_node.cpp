#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <limits>
#include <string>
#include <vector>

#include "cv_bridge/cv_bridge.h"
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/logging.hpp>

// If OpenCV3
#ifndef CV_VERSION_EPOCH
#include <opencv2/imgcodecs.hpp>
#endif

using std::placeholders::_1;

// Compression formats
enum compressionFormat
{
  UNDEFINED = -1, INV_DEPTH
};

// Compression configuration
struct ConfigHeader
{
  // compression format
  compressionFormat format;
  // quantization parameters (used in depth image compression)
  float depthParam[2];
};

class DepthDecodeNode : public rclcpp::Node
{
  public:
    DepthDecodeNode()
    : Node("depth_decode_node")
    {
        this->publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/repub_depth_raw", 10);
        this->subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>("/ros2_camera/depth/image_rect_raw/compressedDepth", 10, std::bind(&DepthDecodeNode::topic_callback, this, _1));
    }

    sensor_msgs::msg::Image::SharedPtr decodeCompressedDepthImage(const sensor_msgs::msg::CompressedImage& message) {
        cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

        auto logger = rclcpp::get_logger("compressed_depth_image_transport");

        // Copy message header
        cv_ptr->header = message.header;

        // Assign image encoding
        std::string image_encoding = message.format.substr(0, message.format.find(';'));
        cv_ptr->encoding = image_encoding;
        if (message.data.size() > sizeof(ConfigHeader)){
            // Read compression type from stream
            ConfigHeader compressionConfig;
            memcpy(&compressionConfig, &message.data[0], sizeof(compressionConfig));

            // Get compressed image data
            const std::vector<uint8_t> imageData(message.data.begin() + sizeof(compressionConfig), message.data.end());

            // Depth map decoding
            float depthQuantA, depthQuantB;

            // Read quantization parameters
            depthQuantA = compressionConfig.depthParam[0];
            depthQuantB = compressionConfig.depthParam[1];
            if (sensor_msgs::image_encodings::bitDepth(image_encoding) == 32)
            {
                cv::Mat decompressed;
                try
                {
                    // Decode image data
                    decompressed = cv::imdecode(imageData, cv::IMREAD_UNCHANGED);
                }
                catch (cv::Exception& e)
                {
                    RCLCPP_ERROR(logger, e.what());
                    return sensor_msgs::msg::Image::SharedPtr();
                }

                size_t rows = decompressed.rows;
                size_t cols = decompressed.cols;

                if ((rows > 0) && (cols > 0))
                {
                    cv_ptr->image = cv::Mat(rows, cols, CV_32FC1);

                    // Depth conversion
                    cv::MatIterator_<float> itDepthImg = cv_ptr->image.begin<float>();
                    cv::MatIterator_<float> itDepthImg_end = cv_ptr->image.end<float>();
                    cv::MatConstIterator_<unsigned short> itInvDepthImg = decompressed.begin<unsigned short>(),
                                                    itInvDepthImg_end = decompressed.end<unsigned short>();

                    for (; (itDepthImg != itDepthImg_end) && (itInvDepthImg != itInvDepthImg_end); ++itDepthImg, ++itInvDepthImg)
                    {
                        // check for NaN & max depth
                        if (*itInvDepthImg)
                        {
                            *itDepthImg = depthQuantA / ((float)*itInvDepthImg - depthQuantB);
                        }
                        else
                        {
                            *itDepthImg = std::numeric_limits<float>::quiet_NaN();
                        }
                    }

                    // Publish message to user callback
                    return cv_ptr->toImageMsg();
                }
            }
            else
            {
                // Decode raw image
                try
                {
                    cv_ptr->image = cv::imdecode(imageData, cv::IMREAD_UNCHANGED);
                }
                catch (cv::Exception& e)
                {
                    RCLCPP_ERROR(logger, e.what());
                    return sensor_msgs::msg::Image::SharedPtr();
                }

                size_t rows = cv_ptr->image.rows;
                size_t cols = cv_ptr->image.cols;

                if ((rows > 0) && (cols > 0))
                {
                    // Publish message to user callback
                    return cv_ptr->toImageMsg();
                }
            }
        }
        return sensor_msgs::msg::Image::SharedPtr();
    }

    

  private:
    void topic_callback(const sensor_msgs::msg::CompressedImage & msg)
    {
        auto thing = decodeCompressedDepthImage(msg);
        publisher_->publish(*thing);
        std::cout << "Got topic" << std::endl;
    }

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthDecodeNode>());
  rclcpp::shutdown();
  return 0;
}