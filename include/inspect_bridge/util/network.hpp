#pragma once

// C++
#include <array>

// Boost
#include <boost/asio.hpp>
#include <boost/algorithm/string.hpp>

// fmt
#include <fmt/core.h>

// json
#include <nlohmann/json.hpp>

using boost::asio::ip::tcp;

namespace ornl::ros::ib::util {
    inline nlohmann::json request(std::string host, std::string port, nlohmann::json command) {
        boost::asio::io_context io_context;

        tcp::resolver resolver(io_context);
        tcp::resolver::results_type endpoints = resolver.resolve(host, port);

        tcp::socket socket(io_context);
        boost::asio::connect(socket, endpoints);

        boost::system::error_code ignored_error;
        boost::asio::write(socket, boost::asio::buffer(command.dump(4)), ignored_error);

        std::string result_str;

        while (true) {
            std::array<char, 128> buf;
            boost::system::error_code error;

            int len = socket.read_some(boost::asio::buffer(buf), error);

            if (error == boost::asio::error::eof) {
                break;
            } else if (error) {
                throw boost::system::system_error(error);
            }

            std::string string_buf = std::string(std::begin(buf), std::begin(buf) + len);
            result_str += string_buf;
        }

        nlohmann::json result = nlohmann::json::parse(result_str);

        return result;
    }
}
