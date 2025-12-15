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
    std::vector<float> data;

    // Pattern to find *each* object
    std::regex object_pattern(
        R"(\{"TAG_ID":\s*(\d+),\s*"ANCHOR_ID":\s*(\d+),\s*"DISTANCE_MEASURED":\s*([\d.]+)\})");

    // Iterators to search the string
    auto begin = std::sregex_iterator(dataLine.begin(), dataLine.end(), object_pattern);
    auto end = std::sregex_iterator();

    RCLCPP_INFO_STREAM(
        this->get_logger(),
        "--- Extracted Data Points ---");
    for (std::sregex_iterator i = begin; i != end; ++i)
    {
        std::smatch match = *i;
        float distance;
        std::string tag_id, anchor_id;

        tag_id = match[1].str();
        anchor_id = match[2].str();
        try
        {
            distance = std::stof(match[3].str());
        }
        catch (...)
        {
            std::cout << "error while parsing" << std::endl;
            return;
        }

        RCLCPP_INFO_STREAM(
            this->get_logger(),
            "TAG_ID:" << tag_id
                      << ", ANCHOR_ID: " << anchor_id
                      << ", DISTANCE: " << distance
            );

        data.push_back(distance);
    }

    RCLCPP_INFO_STREAM(
        this->get_logger(),
        "-----------------------------");

    msg.distances = data;    
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