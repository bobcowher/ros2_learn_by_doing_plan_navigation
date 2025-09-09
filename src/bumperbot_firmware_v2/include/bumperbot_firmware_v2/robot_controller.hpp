// include/robot_driver/robot_controller.hpp
#ifndef ROBOT_DRIVER_ROBOT_CONTROLLER_HPP
#define ROBOT_DRIVER_ROBOT_CONTROLLER_HPP

#include <string>

class RobotController {
private:
    int serial_fd;
    const int DEAD_ZONE = 40;
    
public:
    RobotController();
    ~RobotController();
    
    bool connect(const std::string& port);
    void disconnect();
    bool sendCommand(const std::string& cmd);
    std::string readResponse();
    void moveRobot(int left_speed, int right_speed);
    void stopRobot();
    bool isConnected();
};

#endif // ROBOT_DRIVER_ROBOT_CONTROLLER_HPP
