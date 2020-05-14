#include "ros/ros.h"
#include "std_msgs/String.h"
#include "serial/serial.h"

using serial::Serial;

// Serial stuff
std::string port = "/dev/ttyACM0";
Serial * serial_port;

int main(int argc, char **argv) {

  ros::init(argc, argv, "robo_car_if");

  ros::NodeHandle n;

  ros::Rate loop_rate(1/0.025); // Loop rate in Hz

  // Setup the serial port
  serial_port = new Serial();

  serial_port->setPort(port);
  serial_port->setBaudrate(115200);
  serial_port->setParity(serial::parity_none);
  serial_port->setStopbits(serial::stopbits_one);
  serial_port->setBytesize(serial::eightbits);
  serial::Timeout to = serial::Timeout::simpleTimeout(10);
  serial_port->setTimeout(to);

  // Open the serial port
  serial_port->open();

  if (serial_port->isOpen()) {
    ROS_INFO("Serial port open");
  }

  while (ros::ok()) {
    std_msgs::String msg;
    size_t bytes_avail;

    bytes_avail = serial_port->available();

    if (bytes_avail) {
      serial_port->read((uint8_t *) msg.data.c_str(), bytes_avail);
      ROS_INFO("%s", msg.data.c_str());
    }

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}