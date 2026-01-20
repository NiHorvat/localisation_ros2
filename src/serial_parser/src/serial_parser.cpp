#include <boost/asio.hpp>

#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <chrono>
#include <functional>
#include <regex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "custom_msgs/msg/distances.hpp"

using namespace std;
using namespace boost;

using namespace std::chrono_literals;


class SerialParser : public rclcpp::Node
{
public:
    SerialParser()
        : Node("serial_parser"),
          m_ctx(this->ctx),
          m_port(this->ctx)

    {
        this->declare_parameter("distances_topic", "/distances");
        
        this->declare_parameter("port_name", "/dev/ttyACM0");
        this->declare_parameter("baud_rate", 115200);
        this->declare_parameter("tag_count", 1);
        
        
        publisher_ = this->create_publisher<custom_msgs::msg::Distances>(
            this->get_parameter("distances_topic").as_string(),
            10
        );
        this->configureSerialPort();

        m_io_thread = std::thread([this]()
                                  { this->ctx.run(); });
    }

    void configureSerialPort();

    void readLine();
    void lineReceived(const boost::system::error_code &ec);
    void parseAndPublish(std::string dataLine);

    ~SerialParser();

private:
    asio::io_context ctx;
    asio::io_context &m_ctx;
    asio::serial_port m_port;
    asio::streambuf m_serialData;

    std::thread m_io_thread;

    rclcpp::Publisher<custom_msgs::msg::Distances>::SharedPtr publisher_;


};
void SerialParser::configureSerialPort()
{
    try{

        this->m_port.open(
            this->get_parameter("port_name").as_string());
            this->m_port.set_option(
                asio::serial_port_base::baud_rate(
                    this->get_parameter("baud_rate").as_int()));
                    
                    
    }catch(...){
        
        RCLCPP_ERROR_STREAM(this->get_logger(),
            "Could not open a port " << this->get_parameter("port_name").as_string());
        exit(0);
    }

    // if port opened succesfuly
    this->readLine();

}

void SerialParser::readLine()
{

    async_read_until(this->m_port, this->m_serialData, "\n",
                     [this](const boost::system::error_code &ec,
                            std::size_t size)
                     {
                         (void)size;
                         lineReceived(ec);
                     });
}

void SerialParser::lineReceived(const boost::system::error_code &ec)
{
    if (ec)
        return;

    std::istream is(&m_serialData);

    if (!is.good())
    {
        readLine();
        return;
    }

    std::string s = std::string(
        std::istreambuf_iterator<char>(is),
        std::istreambuf_iterator<char>());

    if (s.at(0) != '$')
    {
        readLine();
        return;
    }

    this->parseAndPublish(s);

    this->readLine();
}
void SerialParser::parseAndPublish(std::string dataLine)
{
    auto msg = custom_msgs::msg::Distances();
    msg.stamp = this->get_clock()->now();
    
    // This regex looks for:
    // 1. "MRA :" followed by digits (Group 1)
    // 2. OR a TAG_ID/ANCHOR_ID/DISTANCE block (Groups 2, 3, and 4)
    std::regex combined_pattern(
        R"(MRA\s*:\s*(\d+)|"TAG_ID":\s*(\d+),\s*"ANCHOR_ID":\s*(\d+),\s*"DISTANCE_MEASURED":\s*([\d.]+))");

    auto begin = std::sregex_iterator(dataLine.begin(), dataLine.end(), combined_pattern);
    auto end = std::sregex_iterator();

    int mra_value = -1;

    for (std::sregex_iterator i = begin; i != end; ++i)
    {
        std::smatch match = *i;

        // Check if Group 1 (MRA) matched
        if (match[1].matched) {
            mra_value = std::stoi(match[1].str());
            RCLCPP_INFO(this->get_logger(), "Found MRA: %d", mra_value);
            msg.mra = mra_value - 2; //FIXME fix stupid code in the dwm3001cdk drivers lol
            continue; 
        }

        // Otherwise, check if Group 2 (The distance object) matched
        if (match[2].matched) {
            try {
                float dist = std::stof(match[4].str());
                msg.distances.push_back(dist);
                
                RCLCPP_INFO(this->get_logger(), "Parsed Point - Tag: %s, Anchor: %s, Dist: %f", 
                             match[2].str().c_str(), match[3].str().c_str(), dist);
            } catch (...) {
                RCLCPP_ERROR(this->get_logger(), "Value conversion error");
            }
        }
    }

    if (mra_value == -1) {
        RCLCPP_WARN(this->get_logger(), "No MRA found in string!");
    }

    this->publisher_->publish(msg);
}
SerialParser::~SerialParser()
{
    this->ctx.stop();
    if (m_io_thread.joinable())
    {
        m_io_thread.join();
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<SerialParser>());

    rclcpp::shutdown();
    return 0;
}