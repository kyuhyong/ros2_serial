#ifndef ASIO_SERIAL_HPP_
#define ASIO_SERIAL_HPP_

#include <iostream>
#include <boost/asio.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <string>
#include <vector>

#define SERIAL_PORT_READ_BUF_SIZE 256

class AsioSerial
{
protected:
    std::shared_ptr<boost::asio::io_service> io_service;
    std::shared_ptr<boost::asio::serial_port> serial;
	boost::mutex mutex_;
	
	std::string read_buf_str_;
	char end_of_read_char_;
	unsigned char rx_buf[SERIAL_PORT_READ_BUF_SIZE];	// For read_some
    boost::asio::streambuf stream_buf_;		// For async_read_until

private:
	AsioSerial(const AsioSerial &p);
	AsioSerial &operator=(const AsioSerial &p); 

public:
	typedef boost::function<void(std::string, int)>CallbackReadUntil;
	CallbackReadUntil cb_read_until_;
	AsioSerial(void);
	virtual ~AsioSerial(void);

    char end_of_read_char() const;
    void set_end_of_read_char(const char &eoc);

	virtual bool open(std::string port_name, int baud_rate=9600);
	virtual void stop();

    int write_some(const std::string &buf);
    void start_async_receive();
	void start_receive();
	void start_async_write(const std::string& message);

	void set_callback_read_until(CallbackReadUntil cb) {
		cb_read_until_ = cb;
	}

protected:
    virtual void handle_write_(const boost::system::error_code& error, std::size_t bytes_transferred);
	virtual void on_receive_(const boost::system::error_code& error, size_t bytes_transferred);
    virtual void on_receive_until_(const boost::system::error_code& error, std::size_t bytes_transferred);

};

#endif