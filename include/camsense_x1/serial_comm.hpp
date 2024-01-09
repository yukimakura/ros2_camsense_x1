// Copyright <2024> [Copyright yukimakura86@gmail.com]
#pragma once

#include <iostream>
#include <iomanip>
#include <boost/thread/thread.hpp>
#include <boost/asio.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <queue>

namespace camsence_x1
{
    class SerialComm
    {
    public:
        SerialComm(const std::string &portname, int baudrate);
        ~SerialComm();
        void ReadStart(std::function<void(unsigned char *,std::size_t)> readCallback);

    private:
        boost::thread thread;
        unsigned char receive_data[128];
        boost::asio::io_service io;
        boost::asio::serial_port serial;
        boost::asio::io_service::work w;
        void readasync(std::function<void(unsigned char *,std::size_t)> readCallback);
    };
}