// Copyright <2020> [Copyright rossihwang@gmail.com]

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <camsense_x1/serial_comm.hpp>

namespace camsence_x1
{

  class CamsenseX1 : public rclcpp::Node
  {
  public:
    CamsenseX1(const std::string &name, rclcpp::NodeOptions const &options);
    ~CamsenseX1();

  private:
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    std::shared_ptr<SerialComm> serial_ptr_;
    std::string frame_id_;
    std::string port_;
    int baud_;
    int angle_offset_;
    unsigned char last_surplus_data_[128];
    size_t last_surplus_data_size_;
    std::thread thread_;
    std::atomic<bool> canceled_;
    uint16_t speed_;
    float start_angle_;
    float end_angle_;
    std::vector<float> ranges_;
    std::vector<float> intensities_;
    rclcpp::Rate rate_;
    OnSetParametersCallbackHandle::SharedPtr callback_handle_;

    void parse(unsigned char *rawbytes,std::size_t rawbyte_size);
    void reset_data();
    void create_parameter();
    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);
    bool handle_parameter(rclcpp::Parameter const &param);
    void deserialize_and_publish(unsigned char *one_scan_data);
  };
}