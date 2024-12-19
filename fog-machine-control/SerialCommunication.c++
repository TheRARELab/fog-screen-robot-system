#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <iostream>
#include <string>
#include <thread>

#define BAUD_RATE 9600

class SerialCommunication {
 public:
  SerialCommunication(boost::asio::io_service& io, const std::string& port_name)
      : io_service_(io), serial_port_(io, port_name) {
    serial_port_.set_option(
        boost::asio::serial_port_base::baud_rate(BAUD_RATE));
  }
  void start();

 private:
  void read_from_port();
  void write_to_port();
  void handle_read(const boost::system::error_code& error, std::size_t bytes_transferred);
  void handle_write(const boost::system::error_code& error, std::size_t bytes_transferred);

  boost::asio::io_service& io_service_;
  boost::asio::serial_port serial_port_;
  boost::asio::streambuf buffer_;
};

void SerialCommunication::start() {
  read_from_port();  // Start reading immediately
  std::thread write_thread(&SerialCommunication::write_to_port, this);
  write_thread.detach();  // Detach the thread to allow it to run independently
}

void SerialCommunication::read_from_port() {
  boost::asio::async_read_until(
      serial_port_, buffer_, "\n",
      boost::bind(&SerialCommunication::handle_read, this,
                  boost::asio::placeholders::error,
                  boost::asio::placeholders::bytes_transferred));
}

void SerialCommunication::write_to_port() {
  while (true) {
    std::string input;
    std::getline(std::cin, input);

    if (input != "exit") {
      input += "\n\r";
      boost::asio::async_write(serial_port_, boost::asio::buffer(input), 
        boost::bind(&SerialCommunication::handle_write, this, 
                    boost::asio::placeholders::error, 
                    boost::asio::placeholders::bytes_transferred));
    } else {
      serial_port_.close();
      io_service_.stop();
      break;  // Exit the loop to end the thread
    }
  }
}

void SerialCommunication::handle_read(const boost::system::error_code& error, std::size_t /*bytes_transferred*/) {
  if (!error) {
    std::istream is(&buffer_);
    std::string line;
    std::getline(is, line);
    std::cout << "Read from port: " << line << std::endl;

    // Continue reading:
    read_from_port();
  } else {
    std::cerr << "Error on receive: " << error.message() << std::endl;
    serial_port_.close();
  }
}

void SerialCommunication::handle_write(const boost::system::error_code& error, std::size_t /*bytes_transferred*/) {
  if (error) {
    std::cerr << "Error on send: " << error.message() << std::endl;
    serial_port_.close();
  }
}

int main() {
  boost::asio::io_service io;
  try {
    SerialCommunication sc(io, "/dev/ttyACM0");
    sc.start();
    io.run();  // This will block and process asynchronous events
  } catch (std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
  }
  return 0;
}
 