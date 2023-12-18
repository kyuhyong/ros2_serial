#include "AsioSerial.hpp"

AsioSerial::AsioSerial(void) : end_of_read_char_('\n')
{
    io_service = std::make_shared<boost::asio::io_service>();
    serial = std::make_shared<boost::asio::serial_port>(*io_service);
}

AsioSerial::~AsioSerial(void)
{
    stop();
}

char AsioSerial::end_of_read_char() const
{
    return this->end_of_read_char_;
}
void AsioSerial::set_end_of_read_char(const char &eoc)
{
    this->end_of_read_char_ = eoc;
}

bool AsioSerial::open(std::string port_name, int baud_rate)
{
    boost::system::error_code ec;
    if(serial->is_open()) {
        std::cout << "Error: port "<<port_name<<" is already opened"<<std::endl;
        return false;
    }
    serial->open(port_name, ec);
    if(ec) {
        std::cout << "Error: open port "<<port_name<<" ec:"<<ec.message().c_str()<<std::endl;
        return false;
    }
    // Set serial port options (baud rate, character size, etc.)
    serial->set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
    serial->set_option(boost::asio::serial_port_base::character_size(8));
    serial->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    
    // Start a thread for async operation
    boost::thread t(boost::bind(&boost::asio::io_service::run, io_service));
    return true;
}

void AsioSerial::stop()
{
    boost::mutex::scoped_lock look(mutex_);
	if (serial) {
		serial->cancel();
		serial->close();
	}
	io_service->stop();
	io_service->reset();
}

int AsioSerial::write_some(const std::string &buf)
{
    const char *tx_buf = buf.c_str();
    const int size = buf.size();
    //boost::system::error_code ec;

    if (!serial) return -1;
    if (size == 0) return 0;

    return this->serial->write_some(boost::asio::buffer(tx_buf, size));
}

void AsioSerial::start_async_receive()
{
    boost::asio::async_read_until(*serial, 
    stream_buf_, "\n",
    boost::bind(&AsioSerial::on_receive_until_,
        this, 
        boost::asio::placeholders::error,
        boost::asio::placeholders::bytes_transferred));
}

void AsioSerial::start_receive()
{
    serial->async_read_some(
        boost::asio::buffer(rx_buf),
        boost::bind(&AsioSerial::on_receive_,
            this, 
            boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred));
}

void AsioSerial::start_async_write(const std::string& message)
{
    boost::asio::async_write(
        *serial, boost::asio::buffer(message),
            boost::bind(&AsioSerial::handle_write_, this,
                boost::asio::placeholders::error,
                boost::asio::placeholders::bytes_transferred));
}

void AsioSerial::handle_write_(const boost::system::error_code& error, std::size_t bytes_transferred)
{
    if (!error) {
        //std::cout << "Write successful!" << std::endl;
        (void)bytes_transferred;
    } else {
        std::cerr << "Write error: " << error.message() << std::endl;
    }
}

void AsioSerial::on_receive_(const boost::system::error_code& error, size_t bytes_transferred)
{
    if(!error) {
        (void)bytes_transferred;
        std::cout << "Received: " << rx_buf<< std::endl;
    } else {
        std::cout << "Error receive"<<std::endl;
        return;
    }
    start_receive();
}

void AsioSerial::on_receive_until_(
    const boost::system::error_code& error, 
    std::size_t bytes_transferred)
{
    if (!error && bytes_transferred) 
    {
        boost::asio::streambuf::const_buffers_type data = stream_buf_.data();
        std::string packet(boost::asio::buffers_begin(data), boost::asio::buffers_end(data));
        //std::cout<<"Read Line:"<<packet<<std::endl;
        stream_buf_.consume(stream_buf_.size());
        if(cb_read_until_) {
            cb_read_until_(packet, bytes_transferred);
        }
        start_async_receive();
    }
    
}