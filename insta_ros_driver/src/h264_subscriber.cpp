#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "h264_msgs/msg/packet.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"

extern "C"
{
#include "libavcodec/avcodec.h"
#include "libswscale/swscale.h"
}

using std::placeholders::_1;

class H264Subscriber : public rclcpp::Node
{
  public:
    H264Subscriber()
    : Node("h264_subscriber")
    {
        image_publisher_ = create_publisher<sensor_msgs::msg::Image>("/output/image", 10);
        p_codec_ = avcodec_find_decoder(AV_CODEC_ID_H264);
        if (!p_codec_) {
            std::cout << "Couldn't find h264 codec" << std::endl;
        } else {
            std::cout << "We are cooking (1/8)" << std::endl;
        }

        p_codec_context_ = avcodec_alloc_context3(p_codec_);
        if (p_codec_context_ == nullptr) {
            std::cout << "Could not alloc codec context" << std::endl;
        } else {
            std::cout << "We are cooking (2/8)" << std::endl;
        }

        if (avcodec_open2(p_codec_context_, p_codec_, nullptr) < 0) {
            std::cout << "Could not open ffmpeg h264 codec" << std::endl;;
        } else {
            std::cout << "We are cooking (3/8)" << std::endl;
        }

        p_packet_ = av_packet_alloc();
        if (p_packet_ == nullptr) {
            std::cout << "Could not alloc packet" << std::endl;
        } else {
            std::cout << "We are cooking (4/8)" << std::endl;
        }

        p_frame_ = av_frame_alloc();
        if (p_frame_ == nullptr) {
            std::cout << "Could not alloc frame" << std::endl;
        } else {
            std::cout << "We are cooking (5/8)" << std::endl;
        }
      h264_subscription_ = this->create_subscription<h264_msgs::msg::Packet>(
      "/input/image", 10, std::bind(&H264Subscriber::camCallback, this, _1));
    }

  private:
    void camCallback(const h264_msgs::msg::Packet::SharedPtr msg)
    {
        
        // for(int i = 0;i < msg->data.size();i++) {
        //     std::cout << msg->data[i];
        // }
        // std::cout << "Something new" << std::endl;

        seq_ = msg->seq;

        p_packet_->size = static_cast<int>(msg->data.size());
        //p_packet_->data = const_cast<uint8_t *>(reinterpret_cast<uint8_t const *>(&msg->data[0]));
        p_packet_->data = msg->data.data();
        // for(int i = 0;i < msg->data.size();i++) {
        //     std::cout << p_packet_->data[i];
        // }
        // std::cout << std::endl;

        if (avcodec_send_packet(p_codec_context_, p_packet_) < 0) {
            std::cout << "Could not send packet" << std::endl;
            return;
        } else {
            std::cout << "We are cooking (6/8)" << std::endl;
        }

        if (avcodec_receive_frame(p_codec_context_, p_frame_) < 0) {
            if (++consecutive_receive_failures_ % 30 == 0) {
                std::cout << "Could not receive frames" << std::endl;
                return;
            }  else {
                std::cout << "We are cooking (7/8)" << std::endl;
            }
        }
        //std::cout << "Consecutive receive failures: " << consecutive_receive_failures_ << std::endl;
        //return;
        if (consecutive_receive_failures_ > 0) {
            std::cout << "Could not receive frames again" << std::endl;
            consecutive_receive_failures_ = 0;
            return;
        } else {
            std::cout << "We are cooking (8/8)" << std::endl;
        }
        

        auto image = std::make_shared<sensor_msgs::msg::Image>();
        image->width = p_frame_->width;
        image->height = p_frame_->height;
        image->step = 3 * p_frame_->width;
        image->encoding = sensor_msgs::image_encodings::BGR8;
        image->header = msg->header;

        // Set / update sws context
        p_sws_context_ = sws_getCachedContext(
            p_sws_context_, p_frame_->width, p_frame_->height, p_codec_context_->pix_fmt,
            p_frame_->width, p_frame_->height, AV_PIX_FMT_BGR24,
            SWS_FAST_BILINEAR, nullptr, nullptr, nullptr);

        // Copy and convert from YUYV420P to BGR24
        image->data.resize(p_frame_->width * p_frame_->height * 3);
        int stride = 3 * p_frame_->width;
        uint8_t * destination = &image->data[0];
        sws_scale(
            p_sws_context_, (const uint8_t * const *) p_frame_->data, p_frame_->linesize, 0,
            p_frame_->height, &destination, &stride);
        image_publisher_->publish(*image);
    }

    rclcpp::Subscription<h264_msgs::msg::Packet>::SharedPtr h264_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    int64_t seq_;
    int consecutive_receive_failures_;
    AVCodec * p_codec_;
    AVCodecContext * p_codec_context_;
    AVFrame * p_frame_;
    AVPacket * p_packet_;
    SwsContext * p_sws_context_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<H264Subscriber>());
  rclcpp::shutdown();
  return 0;
}