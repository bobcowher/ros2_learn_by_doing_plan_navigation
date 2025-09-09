#include <iostream>
#include <string>
#include <chrono>
#include <sys/types.h>
#include <thread>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "bumperbot_firmware_v2/robot_controller.hpp"

RobotController::RobotController() : serial_fd(-1) {}

RobotController::~RobotController() {
	disconnect();
}

bool RobotController::connect(const std::string& port) {
	serial_fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (serial_fd < 0) {
		std::cerr << "Failed to open " << port << std::endl;
		return false;
	}
	
	struct termios tty;
	tcgetattr(serial_fd, &tty);

	cfsetispeed(&tty, B9600);
	cfsetospeed(&tty, B9600);

	// TODO: Go find out what this does. 
	tty.c_cflag |= (CLOCAL | CREAD);
	tty.c_cflag &= ~PARENB;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= CS8;

	tcsetattr(serial_fd, TCSANOW, &tty);

	std::cout << "Waiting for Arduino to initialize..." << std::endl;
	auto start_time = std::chrono::steady_clock::now();

	while(std::chrono::steady_clock::now() - start_time < std::chrono::seconds(5)){
		std::string response = readResponse();
		if(response.find("READY") != std::string::npos){
			std::cout << "Arduino ready!" << std::endl;
			return true;
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}

	std::cerr << "Timeout waiting for Arduino" << std::endl;
	return false;
}

std::string RobotController::readResponse() {
    if (serial_fd < 0) return "";
    
    char buffer[256];
    ssize_t bytes_read = read(serial_fd, buffer, sizeof(buffer) - 1);
    
    if (bytes_read > 0) {
        buffer[bytes_read] = '\0';
        return std::string(buffer);
    }
    return "";
}

void RobotController::moveRobot(int left_speed, int right_speed){
	if (left_speed != 0 && abs(left_speed) < DEAD_ZONE) {
		left_speed = (left_speed > 0) ? DEAD_ZONE : -DEAD_ZONE;
	}
	if (right_speed != 0 && abs(right_speed) < DEAD_ZONE) {
		right_speed = (right_speed > 0) ? DEAD_ZONE : -DEAD_ZONE;
	}

	std::string cmd = "MOVE " + std::to_string(left_speed) + " " + std::to_string(right_speed) + "\n";
	std::cout << "Sending Command: " << cmd;
	sendCommand(cmd);

	std::cout << "Command sent: " << cmd;
}

void RobotController::stopRobot(){
	std::string cmd = "STOP 0 0\n"; 
	sendCommand(cmd);
	std::cout << "Robot Stopped" << std::endl;
}

void RobotController::disconnect(){
	if(serial_fd >= 0){
		close(serial_fd);
		serial_fd = -1;
		std::cout << "Disconnected from robot\n";
	}
}

bool RobotController::sendCommand(const std::string &cmd) {
	if(serial_fd < 0) return false;

	ssize_t bytes_written = write(serial_fd, cmd.c_str(), cmd.length());
	return bytes_written == static_cast<ssize_t>(cmd.length());
}

