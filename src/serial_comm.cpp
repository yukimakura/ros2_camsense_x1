// Copyright <2024> [Copyright yukimakura86@gmail.com]

#include <iostream>
#include <iomanip>
#include <boost/thread/thread.hpp>
#include <boost/asio.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include "camsense_x1/serial_comm.hpp"
using namespace camsence_x1;

SerialComm::SerialComm(const std::string &portname, int baudrate) : io(), serial(io), w(io)
{
    serial.open(portname);
    serial.set_option(boost::asio::serial_port_base::baud_rate(baudrate));
    serial.set_option(boost::asio::serial_port_base::character_size(8));
    serial.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
    serial.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    boost::thread t(boost::bind(&boost::asio::io_service::run, &io));
    thread = boost::move(t);
}

SerialComm::~SerialComm()
{
    io.stop();
    thread.join();
}

void SerialComm::ReadStart(std::function<void(unsigned char *,std::size_t)> readCallback)
{
    this->readasync([&readCallback](unsigned char *data,std::size_t size)
                    { readCallback(data,size); });
}

void SerialComm::readasync(std::function<void(unsigned char *,std::size_t)> readCallback)
{
    boost::asio::async_read(serial, boost::asio::buffer(receive_data),boost::asio::transfer_at_least(40),
                            [this,readCallback](boost::system::error_code ec, std::size_t recv_size)
                            {
                                if (!ec)
                                {
                                    readCallback(receive_data,recv_size);
                                }
                                this->readasync(readCallback);
                            });
}
