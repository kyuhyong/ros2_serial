#include <chrono>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/range.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "nav_msgs/msg/odometry.hpp"

#include "rclcpp/rclcpp.hpp"

#define SERIAL_PORT_READ_BUF_SIZE 256

using namespace std::chrono_literals;

class SerialCom : public rclcpp::Node
{
public:
    SerialCom() : Node("serial_com_node")
    {
        this->configure();
        pub_sensor = this->create_publisher<sensor_msgs::msg::Range>(sensor_topic, 1000);
        tf_broadcaster =std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        io_service = std::make_shared<boost::asio::io_service>();
        serial = std::make_shared<boost::asio::serial_port>(*io_service);
        end_of_line_char_ = 0x0D;
        serial->open(port_name_);

        // Set serial port options (baud rate, character size, etc.)
        serial->set_option(boost::asio::serial_port_base::baud_rate(baudrate_));
        serial->set_option(boost::asio::serial_port_base::character_size(8));
        serial->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        serial->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        
        //boost::thread t(boost::bind(&boost::asio::io_service::run, io_service));
        //std::thread t([io_service](){io_service.run();});
        //sync_read_some_();
        start_receive();
        std::cout<<"Initialize timer"<<std::endl;
        timer_ = this->create_wall_timer(
            timer_ms_, std::bind(&SerialCom::timer_callback, this));
    }

private:
    void timer_callback()
    {
        int count = 0;
        rclcpp::Time now = this->get_clock()->now();
        //this->write_some("Hello\r\n");
        if (count != 0)
        {
            //ROS_INFO_ONCE("Data received from serial port.");
            
            //char read_buf[count];
            //num = ser.read((unsigned char*)read_buf, count);
            // char* ch = strtok(read_buf, ",");
            // if(ch!=NULL && read_buf[0] == '$') {
            //   ch = strtok(NULL, ",");
            //   int dataNum = atoi(ch);
            //   //printf("%s,", ch);
            //   int i = 0;
            //   int sonarData[dataNum] = {0,};
            //   for(i=0; i<dataNum; i++) {
            //     ch = strtok(NULL,",");
            //     sonarData[i] = atoi(ch);
            //     //printf("%d,", sonarData[i]);
            //   }
            //   //printf(" EOF\r\n");
            // }

            //printf("%s\r\n", read_buf);
        }
    }

    int write_some(const std::string &buf)
    {
        const char *tx_buf = buf.c_str();
        const int size = buf.size();
        //boost::system::error_code ec;

        //if (!port_) return -1;
        if (size == 0) return 0;

        return this->serial->write_some(boost::asio::buffer(tx_buf, size));
    }
    /*
    void async_read_some_()
    {
        if (this->serial == NULL || !this->serial->is_open()) return;

        this->serial->async_read_some( 
            boost::asio::buffer(read_buf_raw_, SERIAL_PORT_READ_BUF_SIZE),
            boost::bind(
                static_cast<void(SerialCom::*)(const boost::system::error_code&, size_t)>(&SerialCom::on_receive_),
                this, 
                boost::asio::placeholders::error, 
                boost::asio::placeholders::bytes_transferred));
    }

    void on_receive_(const boost::system::error_code& ec, size_t bytes_transferred)
    {
        boost::mutex::scoped_lock look(mutex_);

        if (this->serial == NULL || !this->serial->is_open()) return;
        if (ec) {
            async_read_some_();
            return;
        }

        for (unsigned int i = 0; i < bytes_transferred; ++i) {
            char c = read_buf_raw_[i];
            if (c == end_of_line_char_) {
                this->on_receive_(read_buf_str_);
                read_buf_str_.clear();
            }
            else {
                read_buf_str_ += c;
            }
        }
        async_read_some_();
    }
    */
    void start_receive()
    {
        this->serial->async_read_some(
            boost::asio::buffer(rx_buf),
            boost::bind(&SerialCom::on_receive_,
                this, 
                boost::asio::placeholders::error,
                boost::asio::placeholders::bytes_transferred));
        this->io_service->run();
    }

    void on_receive_(const boost::system::error_code& error,
        std::size_t bytes_transferred)
    {
        if(!error) {
            std::cout << "Received: " << rx_buf <<", "<<bytes_transferred<< std::endl;
        } else {
            std::cout << "Error receive"<<std::endl;
        }
    }


    void configure()
    {
        this->declare_parameter<std::string>("port.name",       "/dev/ttyUSB0");
        this->declare_parameter<int>        ("port.baudrate",   115200);
        this->declare_parameter<std::string>("sensor.topic",    "sensor_data");
        this->declare_parameter<int>        ("sensor.output_hz", 20);
        this->declare_parameter<bool>       ("tf.enable",       false);
        this->declare_parameter<std::string>("tf.base_frame",   "base_link");
        this->declare_parameter<std::string>("tf.sensor_frame", "sensor_link");

        this->get_parameter<std::string>    ("port.name",       port_name_);
        this->get_parameter<int>            ("port.baudrate",   baudrate_);
        this->get_parameter<int>            ("sensor.output_hz", output_hz_);
        this->get_parameter<std::string>    ("sensor.topic",     sensor_topic);
        this->get_parameter<bool>           ("tf.enable",       is_pub_tf);
        this->get_parameter<std::string>    ("tf.base_frame",   base_frame);
        this->get_parameter<std::string>    ("tf.sensor_frame",  sensor_frame);
        RCLCPP_INFO(this->get_logger(), "Params:");
        RCLCPP_INFO(this->get_logger(), "\tport.Name: %s",      port_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "\tport.Baudrate: %d",  baudrate_);
        RCLCPP_INFO(this->get_logger(), "\tSENSOR.Ouput_Hz: %d",   output_hz_);
        timer_ms_ = std::chrono::milliseconds {static_cast<long int>(1000 / output_hz_)};
        RCLCPP_INFO(this->get_logger(), "\tMilliseconds: %d",     timer_ms_.count());
        RCLCPP_INFO(this->get_logger(), "\tSENSOR.Topic: %s",     sensor_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "\tTF.Base_link: %s",     base_frame.c_str());
        RCLCPP_INFO(this->get_logger(), "\tTF.Sensor_link: %s",   sensor_frame.c_str());
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr       pub_sensor;
    std::string port_name_;
    int baudrate_;
    int pub_rate_;
    int output_hz_;
    std::shared_ptr<boost::asio::io_service> io_service;
    //boost::asio::io_context io_context;
    std::shared_ptr<boost::asio::serial_port> serial;
    char end_of_line_char_;
    char read_buf_raw_[SERIAL_PORT_READ_BUF_SIZE];
	//std::string read_buf_str_;
    unsigned char rx_buf[SERIAL_PORT_READ_BUF_SIZE];
    boost::mutex mutex_;

    bool is_pub_tf;
    std::chrono::milliseconds timer_ms_;
    std::string sensor_frame;
    std::string base_frame;
    std::string sensor_topic;
    std::unique_ptr<tf2_ros::TransformBroadcaster>              tf_broadcaster;
};

int main(int argc, char * argv[])
{
    std::cout << "Starting serial com node..." << std::endl;
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialCom>());
    rclcpp::shutdown();
    return 0;
}
