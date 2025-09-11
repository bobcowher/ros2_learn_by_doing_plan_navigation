// include/robot_driver/robot_controller.hpp
#ifndef ROBOT_DRIVER_ROBOT_CONTROLLER_HPP
#define ROBOT_DRIVER_ROBOT_CONTROLLER_HPP

#include <string>
#include <cmath>

class RobotController {
private:
    int serial_fd;
    const int DEAD_ZONE = 40;

    // Physical constants for your robot
    const double WHEEL_DIAMETER = 0.07;        // 7cm in meters
    // const double PI = 3.14159265358979323846; // Come back and fix this.  
    const double WHEEL_CIRCUMFERENCE = M_PI * WHEEL_DIAMETER;  // ~0.22 meters
    const double MOTOR_MAX_RPM = 100;
    const int MOTOR_MAX_PWM = 255;
    const double MAX_ACCELERATION = 0.5;  // m/s² 
    const double CONTROL_RATE = 50.0;     // Hz (how often we get commands)
    const double MAX_VELOCITY_CHANGE = MAX_ACCELERATION / CONTROL_RATE;

    // State tracking
    double last_left_velocity = 0.0;
    double last_right_velocity = 0.0;
    
    double applyAccelerationLimit(double target_vel, double current_vel);
    
    // Your robot's theoretical max speed:
    // 100 RPM = 100/60 = 1.67 rotations per second
    // 1.67 * 0.22m = 0.367 m/s max speed
    const double MAX_VELOCITY = (MOTOR_MAX_RPM / 60.0) * WHEEL_CIRCUMFERENCE;  // 0.367 m/s
    
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
    int velocityToMotorSpeed(double velocity_ms);
    void moveRobotVelocity(double left_vel_ms, double right_vel_ms); 
};

#endif // ROBOT_DRIVER_ROBOT_CONTROLLER_HPP
