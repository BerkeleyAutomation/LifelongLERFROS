// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>
#include <typeinfo>
#include <cxxabi.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <iostream>
#include <thread>
#include <camera/camera.h>
#include <camera/photography_settings.h>
#include <camera/device_discovery.h>
#include "h264_msgs/msg/packet.hpp"
#include <regex>
#include <cmath>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

class InstaFisheyePublisher : public ins_camera::StreamDelegate, public rclcpp::Node {
public:
    InstaFisheyePublisher(): Node("insta_fisheye_publisher"), count_(0) {
        file1_ = fopen("./optimus.h264", "wb");
        file2_ = fopen("./megatron.h264", "wb");

        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        h264_pub1_ = create_publisher<h264_msgs::msg::Packet>("image_raw/h264_1", 10);
        h264_pub2_ = create_publisher<h264_msgs::msg::Packet>("image_raw/h264_2", 10);
        //timer_ = this->create_wall_timer(
        //500ms, std::bind(&InstaFisheyePublisher::timer_callback, this));

        std::cout << "begin open camera" << std::endl;
        ins_camera::DeviceDiscovery discovery;
        auto list = discovery.GetAvailableDevices();
        for (int i = 0; i < list.size(); ++i) {
            auto desc = list[i];
            std::cout << "HiCam device found:" << std::endl;
            std::cout << "serial:" << desc.serial_number << "\t"
                << "camera type:" << int(desc.camera_type) << "\t"
                << "lens type:" << int(desc.lens_type) << std::endl;
        }

        if (list.size() <= 0) {
            std::cerr << "no device found." << std::endl;
            return;
        }

        cam = std::make_shared<ins_camera::Camera>(list[0].info);
        //ins_camera::Camera cam(list[0].info);
        if (!cam->Open()) {
            std::cerr << "failed to open camera" << std::endl;
            return;
        }

        std::cout << "http base url:" << cam->GetHttpBaseUrl() << std::endl;

        std::shared_ptr<ins_camera::StreamDelegate> delegate = std::make_shared<InstaFisheyePublisher>();
        cam->SetStreamDelegate(delegate);

        discovery.FreeDeviceDescriptors(list);

        std::cout << "Succeed to open camera..." << std::endl;

        auto camera_type = cam->GetCameraType();

        auto start = time(NULL);
        cam->SyncLocalTimeToCamera(start);

        int option;

        ins_camera::LiveStreamParam param;
        param.video_resolution = ins_camera::VideoResolution::RES_720_360P30;
        param.lrv_video_resulution = ins_camera::VideoResolution::RES_720_360P30;
        param.video_bitrate = 1024 * 1024 / 2;
        param.enable_audio = false;
        param.using_lrv = false;
        if (cam->StartLiveStreaming(param)) {
            std::cout << "successfully started live stream" << std::endl;
        }
    }
    ~InstaFisheyePublisher() {
        fclose(file1_);
        fclose(file2_);
        if (cam->StopLiveStreaming()) {
            std::cout << "success!" << std::endl;
        }
        else {
            std::cerr << "failed to stop live." << std::endl;
        }
        std::cout << "Destructed" << std::endl;
    }

    void OnAudioData(const uint8_t* data, size_t size, int64_t timestamp) override {
        std::cout << "on audio data:" << std::endl;
    }
    void OnVideoData(const uint8_t* data, size_t size, int64_t timestamp, uint8_t streamType, int stream_index = 0) override {
        /*for(int i = 0;i < size;i++) {
            std::cout << data[i];
        }*/
        h264_msgs::msg::Packet h264_msg;
        h264_msg.header.frame_id = "camera_frame";
        h264_msg.seq = seq_++;
        //h264_msg.data.insert(h264_msg.data.end(), data[0], data[size]);
        uint8_t test_data[] = {1,2,3,4,5};
        uint8_t *test_data_ptr = test_data;
        // for(int i = 0;i < 5;i++) {
        //     std::cout << test_data_ptr[i];
        // }
        // std::cout << std::endl;
        //h264_msg.data.insert(h264_msg.data.end(),test_data_ptr[0],test_data_ptr[5]);
        h264_msg.data.insert(h264_msg.data.end(), &data[0], &data[size]);
        auto stamp = rclcpp::Node::now();
        h264_msg.header.stamp = stamp;
        if (stream_index == 0) {
            fwrite(data, sizeof(uint8_t), size, file1_);
            h264_pub1_->publish(h264_msg);
        }
        if (stream_index == 1) {
            fwrite(data, sizeof(uint8_t), size, file2_);
            h264_pub2_->publish(h264_msg);
        }
    }
    void OnGyroData(const std::vector<ins_camera::GyroData>& data) override {
        //for (auto& gyro : data) {
        //	if (gyro.timestamp - last_timestamp > 2) {
        //		fprintf(file1_, "timestamp:%lld package_size = %d  offtimestamp = %lld gyro:[%f %f %f] accel:[%f %f %f]\n", gyro.timestamp, data.size(), gyro.timestamp - last_timestamp, gyro.gx, gyro.gy, gyro.gz, gyro.ax, gyro.ay, gyro.az);
        //	}
        //	last_timestamp = gyro.timestamp;
        //}
    }
    void OnExposureData(const ins_camera::ExposureData& data) override {
        //fprintf(file2_, "timestamp:%lld shutter_speed_s:%f\n", data.timestamp, data.exposure_time);
    }

private:
    FILE* file1_;
    FILE* file2_;
    int64_t last_timestamp = 0;
    int seq_{};
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Publisher<h264_msgs::msg::Packet>::SharedPtr h264_pub1_;
    rclcpp::Publisher<h264_msgs::msg::Packet>::SharedPtr h264_pub2_;
    size_t count_;
    std::shared_ptr<ins_camera::Camera> cam;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InstaFisheyePublisher>());
  rclcpp::shutdown();
  return 0;
}
