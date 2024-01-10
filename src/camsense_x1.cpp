// Copyright <2020> [Copyright rossihwang@gmail.com]
// Modified by yukimakura <2024> [Copyright yukimakura86@gmail.com]

#include <camsense_x1/camsense_x1.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <cmath>
using namespace camsence_x1;

constexpr uint8_t kSync1 = 0x55;
constexpr uint8_t kSync2 = 0xaa;
constexpr uint8_t kSync3 = 0x03;
constexpr uint8_t kSync4 = 0x08;
constexpr uint8_t kChunksize = 36;
constexpr double kIndexMultiplier = 400 / 360;

CamsenseX1::CamsenseX1(const std::string &name, rclcpp::NodeOptions const &options)
    : Node(name, options),
      frame_id_("scan"),
      port_("/dev/ttyUSB0"),
      baud_(115200),
      angle_offset_(0),
      canceled_(false),
      speed_(0),
      start_angle_(0.0),
      end_angle_(0.0),
      last_surplus_data_size_(0),
      rate_(std::chrono::milliseconds(32))
{
  try
  {

    RCLCPP_INFO(this->get_logger(), "create_parameter");
    create_parameter();

    RCLCPP_INFO_STREAM(this->get_logger(), "param frame_id_: " << frame_id_);
    RCLCPP_INFO_STREAM(this->get_logger(), "param port_: " << port_);
    RCLCPP_INFO_STREAM(this->get_logger(), "param baud_: " << baud_);
    RCLCPP_INFO_STREAM(this->get_logger(), "param angle_offset_: " << angle_offset_);

    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
    serial_ptr_ = std::make_shared<SerialComm>(port_, baud_);

    RCLCPP_INFO(this->get_logger(), "%s is open()", port_.c_str());

    serial_ptr_->ReadStart([this](unsigned char *rawbytes, std::size_t recv_size)
                           { this->parse(rawbytes, recv_size); });
    ranges_.resize(400);
    intensities_.resize(400);

    RCLCPP_INFO(this->get_logger(), "reset_data");
    reset_data();

    while (rclcpp::ok() && !canceled_.load())
    {
      // parse();
      rate_.sleep();
    }
  }
  catch (std::exception &ex)
  {
    RCLCPP_ERROR(this->get_logger(), ex.what());
  }
}

CamsenseX1::~CamsenseX1()
{
  canceled_.store(true);
  if (thread_.joinable())
  {
    thread_.join();
  }
}

void CamsenseX1::parse(unsigned char *rawbytes, std::size_t rawbyte_size)
{

  unsigned char rawbytes_with_surplus_data[rawbyte_size + last_surplus_data_size_];

  int rawbytes_with_surplus_data_count = rawbyte_size + last_surplus_data_size_;
  std::copy(last_surplus_data_, last_surplus_data_ + last_surplus_data_size_, rawbytes_with_surplus_data);
  std::copy(rawbytes, rawbytes + rawbyte_size, rawbytes_with_surplus_data + last_surplus_data_size_);

  if (rawbytes_with_surplus_data_count < 40)
  {
    last_surplus_data_size_ = rawbytes_with_surplus_data_count;
    std::copy(rawbytes_with_surplus_data, rawbytes_with_surplus_data + last_surplus_data_size_, last_surplus_data_);
    return;
  }
  int processed_data_index = 0;
  // split
  for (size_t i = 0; i < (rawbytes_with_surplus_data_count - 4); i++)
  {
    if (rawbytes_with_surplus_data[i] == kSync1 &&
        rawbytes_with_surplus_data[i + 1] == kSync2 &&
        rawbytes_with_surplus_data[i + 2] == kSync3 &&
        rawbytes_with_surplus_data[i + 3] == kSync4)
    {
      if ((rawbytes_with_surplus_data_count - i) > kChunksize)
      {
        unsigned char chunkdata[kChunksize];
        std::copy(rawbytes_with_surplus_data + i, rawbytes_with_surplus_data + i + kChunksize, chunkdata);
        deserialize_and_publish(chunkdata);
        processed_data_index = i + kChunksize;
      }
    }
  }

  if ((rawbytes_with_surplus_data_count - processed_data_index) > 0)
  {
    last_surplus_data_size_ = rawbytes_with_surplus_data_count - processed_data_index;
    std::copy(rawbytes_with_surplus_data + processed_data_index, rawbytes_with_surplus_data + processed_data_index + (rawbytes_with_surplus_data_count - processed_data_index), last_surplus_data_);
  }
}

void CamsenseX1::deserialize_and_publish(unsigned char *one_scan_data)
{
  // case State::SPEED:
  speed_ = static_cast<uint16_t>(one_scan_data[5]) << 8 | one_scan_data[4];
  // case State::START:
  uint16_t startcount;
  startcount = static_cast<uint16_t>(one_scan_data[7]) << 8 | one_scan_data[6];
  start_angle_ = startcount / 64.0 - 640;
  // case State::DATA:
  float angle_res;
  for (int i = 0; i < 8; ++i)
  {
    int j = (3 * i) + 8;
    uint16_t range = static_cast<uint16_t>(one_scan_data[j + 1]) << 8 | one_scan_data[j];
    uint8_t intensity = one_scan_data[j + 2];
    double measured_angle = start_angle_ + angle_res * i - angle_offset_;
    int angle_index = std::round(measured_angle * kIndexMultiplier);
    angle_index %= 400;
    angle_index = 399 - angle_index;
    if (range == 0x8000)
    {
      range = 8000; // maximum range in mm
    }
    ranges_[angle_index] = static_cast<float>(range) / 1000.0;
    intensities_[angle_index] = static_cast<float>(intensity);
    // case State::END:
    uint16_t endcount;
    endcount = static_cast<uint16_t>(one_scan_data[33]) << 8 | one_scan_data[32];
    end_angle_ = endcount / 64.0 - 640;

    // TODO: validate crc
    if (end_angle_ < start_angle_)
    {
      angle_res = (end_angle_ + 360 - start_angle_) / 8.0;
    }
    else
    {
      angle_res = (end_angle_ - start_angle_) / 8.0;
    }
  }
  if (end_angle_ < start_angle_)
  {
    sensor_msgs::msg::LaserScan message;
    message.header.stamp = now();
    message.header.frame_id = frame_id_;
    message.angle_increment = (2.0 * M_PI) / 400.0;
    message.angle_min = 0.0;
    message.angle_max = 2.0 * M_PI - message.angle_increment;
    message.scan_time = 0.001;
    message.range_min = 0.08; // camsense x1 spec
    message.range_max = 8;    // camsense x1 spec
    message.ranges = ranges_;
    message.intensities = intensities_;
    scan_pub_->publish(message);
    reset_data();
    rate_.sleep();
  }
}

void CamsenseX1::reset_data()
{
  for (int i = 0; i < 400; ++i)
  {
    ranges_[i] = 8;
    intensities_[i] = 0;
  }
}

void CamsenseX1::create_parameter()
{
  frame_id_ = declare_parameter<std::string>("frame_id", "scan");
  port_ = declare_parameter<std::string>("port", "/dev/ttyUSB0");
  baud_ = declare_parameter<int>("baud", 115200);
  angle_offset_ = declare_parameter<int>("angle_offset", 0);

  callback_handle_ = this->add_on_set_parameters_callback(std::bind(&CamsenseX1::parametersCallback, this, std::placeholders::_1));
}
rcl_interfaces::msg::SetParametersResult CamsenseX1::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters)
{
  rcl_interfaces::msg::SetParametersResult result;

  for (const auto &param : parameters)
  {
    RCLCPP_INFO(this->get_logger(), "%s", param.get_name().c_str());
    if (param.get_name() == "frame_id")
    {
      frame_id_ = param.as_string();
    }
    else if (param.get_name() == "port")
    {
      port_ = param.as_string();
    }
    else if (param.get_name() == "baud")
    {
      baud_ = param.as_int();
    }
    else if (param.get_name() == "rotation")
    {
      angle_offset_ = param.as_int() % 360;
    }
    else
    {
      result.successful = false;
      result.reason = "invalid parameter :" + param.get_name();
      return result;
    }
  }
  result.successful = true;
  result.reason = "success";
  return result;
}